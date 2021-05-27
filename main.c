/*
 * UWComp.c
 *
 * Created: 27.05.2020 23:27:06
 * Author : Drumir
 */ 
#include "global.h"
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "util/delay.h"
#include "uwcomp.h"
#include "font.h"
#include "lcd.h"
#include "DS3231_twi.h"
#include "DS18B20.h"

void measureBattery(void);            // Измеряет напряжение на аккумуляторе. Результат в Vbat
void measurePressure(void);           // Измеряет мгновенное давление. Помещает результат в PressArray[PressArrayCount]
void measurePressureInSleep(void);    // Запускает измерение мгновенного давления используя ADC noise reduction. результат обрабатывает ISR(ADC_vect)
uint16_t GetAveragePressure(void);    // Возвращает среднее арехметическое давление по массиву PressArray[]
void DrawMenu(void);
void DrawAir(void);
void DrawDeep(void);
void DrawSurface(void);
void Sleep(void);
uint16_t PressureToDepth(uint16_t p);   // Возвращает вычисленную по давлению глубину в сантиметрах

char str[10];
//uint8_t ch2 = 0b01010101;
int8_t Mode, menuItem;
int16_t CurrentModeLeft;
uint8_t hour, min, sec, rtc_temp;
uint8_t contrast;
uint16_t sqw, Vbat, Temp, Depth, Hold, LastDepth, LastHold;  // Глубины в сантиметрах
uint16_t Pressure; // Давление в сотнях паскалей
uint16_t MinPressure; // Минимальное измеренное давленее (Примем его за атмосферное)
uint16_t PressArray[PRESS_ARRAY_LENGTH];
uint8_t PressArrayCount;

int main(void)
{
	DDRB  = 0b01101111;		//KeyRight, BT, SCK, MISO, MOSI, LCD_CSE, LCD_DC, LCD_RESET
	PORTB = 0b11000000;   // Подтяжка на геркон, BT(0-on/1-off), ,,,,,,
	
	DDRD  = 0b10000011;			// BTrst, KeyUp, KeyLeft, DS18B20, INT1(Keys), INT0, TX, LED
	PORTD = 0b01101000;			// 
  
	DDRC  = 0b00000001;   // ,,,,,,,5V
	PORTC = 0b00000000;   // ,,,,,,,5V(0-on/1-off)
  
	ACSR |= 0b10000000; // Выключим аналоговый компаратор
  
	DIDR0 |= 0b00000010;  // Отключим входной цифровой буфер на PC1 (ADC1) (на ADC6 и ADC7 цифровой буфер отсутствует)
  
	EICRA |= (1<<ISC01) | (0<<ISC00) | (1<<ISC11) | (0<<ISC10); // The rising edge of INT0 and INT1 generates an interrupt request
	EIMSK |= (1<<INT0) | (1<<INT1); // INT0 Enable, INT1 Enable

  /* Инициализация АЦП */
  //         АЦП En,    not now,  single mode, reset iflag, INTs Disable,       предделитель частоты
  ADCSRA = 1 << ADEN | 0 << ADSC | 0 << ADATE | 0 << ADIF | 0 << ADIE | 1 << ADPS2 | 1 << ADPS1 | 1 << ADPS0;
  //	ADCSRA = 0b10001111;		// 1 - вкл, 0 - еще не старт, 0 - однократно, 0 - прерывания генерировать, 0 - прерывания АЦП разрешить, 111 - предделитель частоты 128
  ADMUX  = 0b11000111;		// 11 - Опорное напряжение = 1.1В, 0 - выравнивание вправо, 0 - резерв, 0 - резерв, 000 - выбор канала ADC7
  ADCSRA |= 1<<ADSC;		// Старт пробного мусорного преобразования

  LastDepth = 0;
	LastHold = 0;
	//rtc_set_time(21, 41, 6);
  //rtc_set_date(28, 2, 21);
	rtc_write(0x0E, 0b01000000);				// Запуск меандра 1 Гц
  
  Mode = M_MENU;                      // Комп включается в режиме меню
  CurrentModeLeft = MENU_MODE_TIME;   // 
  menuItem = MI_Contrast;             // Выделенный пункт меню
  
  lcd_init(LCD_DISP_ON);              // init lcd and turn on
  contrast = 128;
  lcd_set_contrast(contrast);
  
  rtc_get_time(&hour, &min, &sec);    // Прочитаем текущее время
  
  _delay_ms(300);
  EIFR &= ~(0<<INT1);                  // Сбросим флаг преждевременного прерывания от клавы
  
  PressArrayCount = 0;
  for(uint8_t i = 0; i < PRESS_ARRAY_LENGTH; i ++)
    measurePressure();                  // Наполним массив измерений давления чтобы усредненное давление стало актуальным
	MinPressure = 1100;
  
	sei();
	
  while (1) 
  {
    for(uint8_t i = 0; i < PRESS_ARRAY_LENGTH; i ++) measurePressure();
    Pressure = GetAveragePressure();
		if(Pressure < MinPressure) MinPressure = Pressure;
    measureBattery();
    
    if(Mode != M_DEEP && PressureToDepth(Pressure) > 99){
      Mode = M_DEEP;
      Hold = 0;
			LastDepth = 0;
    }

    if(Mode == M_DEEP && PressureToDepth(Pressure) <= 99){ // Здесь должен быть не M_AIR, а M_SURFACE
      Mode = M_AIR;
			LastHold = Hold;
    }

    switch(Mode){
      case M_MENU:{
        DrawMenu();
        break;
      }
      case M_AIR:{
        DrawAir();
        break;
      }
      case M_DEEP:{
				if (Depth > LastDepth) LastDepth = Depth;
        DrawDeep();
        break;
      }
      case M_SURFACE:{
        DrawSurface();
        break;
      }
      case M_SLEEP:{
        Sleep();
        break;
      }
    }
  }
}

//--------------------------------------------------------------

ISR(INT0_vect)                    // Ежесекундное прерывание от RTC
{
  if(CurrentModeLeft == 0){
    if(Mode == M_MENU){
      Mode = M_AIR;
      CurrentModeLeft = AIR_MODE_TIME;
    }
    if(Mode == M_AIR){
      //Mode = M_SLEEP; // Переход в спящий режим будет происходить в главном цикле (чтоб не в прерывании)
      CurrentModeLeft = -1;
    }
  }    

  if(CurrentModeLeft > 0)
    CurrentModeLeft --;
    
  if(Mode == M_DEEP)
    Hold ++;
  
  sec ++;
  if (sec == 60)
  {
    rtc_get_time(&hour, &min, &sec); // Время из RTC читаем лишь раз в минуту для экономии
  }
  
//  rtc_temp = rtc_read(0x11); // Чтение целой части температуры RTC
  
	sei();              // 
  
  if(sec%2 == 0){
    sensor_write(0x44);   // старт измерения температуры
		PORTD |= 0b00000001;			// Поморгаем светодиодом
	}
  else{
    Temp = sensor_write(0xBE); // чтение температурных данных c dc18_B_20 / dc18_S_20
		PORTD &= 0b11111110;			//
  }       
	return;
}
//------------------------------------------------------------------------------

ISR(INT1_vect)                  // Прерывание от кнопок
{
  Mode = M_MENU;
  CurrentModeLeft = MENU_MODE_TIME;
  if((PINB & 0b10000000) == 0)            // Key RIGHT
  {
    menuItem ++;
    if(menuItem == MI_Last) menuItem = MI_TimeH;
  }
  else if((PIND & 0b01000000) == 0)       // Key UP
  {
    switch(menuItem){
      case MI_TimeH:{
        hour ++;
        if(hour > 23) hour = 0;
        rtc_set_time(hour, min, 0);
        break;
      }
      case MI_TimeM:{
        min ++;
        if(min > 59) min = 0;
        rtc_set_time(hour, min, 0);
        break;
      }
      case MI_Contrast:{
        contrast += 8;
        //if(contrast > 255) contrast = 0;
        lcd_set_contrast(contrast);
        break;
      }
    }
  }
  else if((PIND & 0b00100000) == 0)       // Key LEFT
  {
    menuItem --;
    if(menuItem < 0) menuItem = MI_Last-1;
  }
  else                                // Key DOWN
  {
    switch(menuItem){
      case MI_TimeH:{
        hour --;
        if(hour > 23) hour = 23;
        rtc_set_time(hour, min, 0);
        break;
      }
      case MI_TimeM:{
        min --;
        if(min > 59) min = 59;
        rtc_set_time(hour, min, 0);
        break;
      }
      case MI_Contrast:{
	      contrast -= 8;
	      //if(contrast < 0) contrast = 255;
	      lcd_set_contrast(contrast);
	      break;
      }
      case MI_ToSleep:{
		  Mode = M_SLEEP;
	      break;
      }
    }
  }    
	return;
}
//------------------------------------------------------------------------------

ISR(ADC_vect)          // Завершение преобразования АЦП
{
  SMCR &= ~(1 << SE);  // Сбросим флаг готовности к спящему режиму
  uint32_t vLongPress = ADC;
  vLongPress *= 540764;   // Здесь мы получаем напряжение на PC1 умноженное на миллион вроде
  
  //vLongPress /= 100;
  vLongPress /= 11829;
  vLongPress *= 100;
  vLongPress += 34779;
  //PressArray[0] = vLongPress/1000;
  
  PressArrayCount ++;
  if(PressArrayCount == PRESS_ARRAY_LENGTH) PressArrayCount = 0;
  PressArray[PressArrayCount] = vLongPress/1000; // Получаем давление в сотнях паскалей (hPa)
}
//---------------------------------------------------------------------
void measurePressureInSleep(void)    // Измеряет давление в hPa
{
  ADMUX  = 0b11000001;		  // 11 - Опорное напряжение = 1.1В, 0 - выравнивание вправо, 0 - резерв, 0 - резерв, 001 - выбор канала ADC1
  //         АЦП En,    not now,  single mode, reset iflag, INTs Enable,       предделитель частоты
  ADCSRA = 1 << ADEN | 0 << ADSC | 0 << ADATE | 0 << ADIF | 1 << ADIE | 1 << ADPS2 | 1 << ADPS1 | 1 << ADPS0;
  SMCR |= 1 << SE | 0 << SM2 | 0 << SM1 | 1 << SM0;      // Разрешим переход в спящий режим. Выберем режим ADC Noise Reduction.

  sleep_mode();
}
//---------------------------------------------------------------------
void measurePressure(void)    // Измеряет давление в hPa
{
  //         АЦП En,    not now,  single mode, reset iflag, INTs Disable,       предделитель частоты
  ADCSRA = 1 << ADEN | 0 << ADSC | 0 << ADATE | 0 << ADIF | 0 << ADIE | 1 << ADPS2 | 1 << ADPS1 | 1 << ADPS0;
  ADMUX  = 0b11000001;		  // 11 - Опорное напряжение = 1.1В, 0 - выравнивание вправо, 0 - резерв, 0 - резерв, 001 - выбор канала ADC1
  ADCSRA |= 1<<ADSC;		    // Старт преобразования
  while (ADCSRA & 0x40);		// Ждем завершения(сброса флага ADSC в 0)
  uint32_t vLongPress = ADC;
  vLongPress *= 540764;   // Здесь мы получаем напряжение на PC1 умноженное на миллион вроде
  vLongPress /= 11829;
  vLongPress *= 100;
  vLongPress += 34779;
  PressArrayCount ++;
  if(PressArrayCount == PRESS_ARRAY_LENGTH) PressArrayCount = 0;
  PressArray[PressArrayCount] = vLongPress/1000; // Получаем давление в сотнях паскалей (hPa)
}
//------------------------------------------------------------------------------
void measureBattery(void)
{
  //         АЦП En,    not now,  single mode, reset iflag, INTs Disable,       предделитель частоты
  ADCSRA = 1 << ADEN | 0 << ADSC | 0 << ADATE | 0 << ADIF | 0 << ADIE | 1 << ADPS2 | 1 << ADPS1 | 1 << ADPS0;
  ADMUX  = 0b11000111;		  // 11 - Опорное напряжение = 1.1В, 0 - выравнивание вправо, 0 - резерв, 0 - резерв, 111 - выбор канала ADC7
  ADCSRA |= 1<<ADSC;		    // Старт преобразования
  while (ADCSRA & 0x40);		// Ждем завершения(сброса флага ADSC в 0)
  uint32_t vLongBat = ADC;
  vLongBat *= 1000;
  Vbat = vLongBat / 2291;
}
//---------------------------------------------------------------------
void DrawMenu(void)
{
	lcd_charMode(DOUBLESIZE);
	lcd_gotoxy(0, 0);
  lcd_puts("Time");
	lcd_putc(' ');
  if(menuItem == MI_TimeH)lcd_inversMode(INVERS);
	lcd_putc('0' + hour/10);
	lcd_putc('0' + hour%10);
  lcd_inversMode(NOINVERS);
	lcd_putc(':');
  if(menuItem == MI_TimeM)lcd_inversMode(INVERS);
	lcd_putc('0' + min/10);
	lcd_putc('0' + min%10);
  lcd_inversMode(NOINVERS);
	lcd_gotoxy(0, 2);
  lcd_puts("Cntrst");
	lcd_putc(' ');
  if(menuItem == MI_Contrast)lcd_inversMode(INVERS);
  itoa(contrast/8, str, 10);
  lcd_puts(str);
	lcd_putc(' ');
  lcd_inversMode(NOINVERS);
	lcd_gotoxy(0, 4);
  lcd_puts("Sleep");
  lcd_putc(' ');
  if(menuItem == MI_ToSleep)lcd_inversMode(INVERS);
  lcd_puts(" NO");
  lcd_inversMode(NOINVERS);
  
}
//---------------------------------------------------------------------
void DrawAir(void)
{
	lcd_charMode(DOUBLESIZE);
  lcd_gotoxy(0, 0);               // Отобразим задержку предыдущего нырка
  if(LastHold < 600) lcd_putc('0');
  itoa(LastHold/60, str, 10);
  lcd_puts(str);
  lcd_putc(':');
  if(LastHold%60 < 10) lcd_putc('0');
  itoa(LastHold%60, str, 10);
  lcd_puts(str);
  lcd_putc(' ');
	lcd_gotoxy(11, 0);

	itoa(LastDepth, str, 10);			// Отобразим макс глубину предыдущего нырка
	if (LastDepth >= 1000 ){
		lcd_putc(str[0]);
		lcd_putc(str[1]);
		lcd_putc('.');
		lcd_puts(str+2);
	}
	else if(LastDepth >= 100){
		lcd_putc(str[0]);
		lcd_putc('.');
		lcd_puts(str+1);
//	lcd_charMode(NORMALSIZE);
	lcd_putc('m');
//	lcd_charMode(DOUBLESIZE);
	}
	else{
		lcd_putc('0');
		lcd_putc('.');
		lcd_puts(str);
	}
//	lcd_puts("m ");


  itoa(Temp/16, str, 10);
  lcd_gotoxy(0, 2);
  lcd_putsB(str);
  lcd_putsB("C    ");

  lcd_gotoxy(0, 5);
  lcd_puts("              ");
  
  lcd_charMode(DOUBLESIZE);
  lcd_gotoxy(0, 6);
  lcd_putc('0' + hour/10);
  lcd_putc('0' + hour%10);
  lcd_putc(':');
  lcd_putc('0' + min/10);
  lcd_putc('0' + min%10);
  lcd_putc(' ');

  itoa(Vbat, str, 10);
  lcd_puts(str);
  lcd_putc('v');
}

//---------------------------------------------------------------------
void DrawDeep(void)
{
  lcd_gotoxy(0, 0);               // Отобразим задержку
  lcd_putcB('_');
  if(Hold < 600) lcd_putcB('0');
  itoa(Hold/60, str, 10);
  lcd_putsB(str);
  lcd_putcB(':');
  if(Hold%60 < 10) lcd_putcB('0');
  itoa(Hold%60, str, 10);
  lcd_putsB(str);
  lcd_putcB('_');


  lcd_gotoxy(0, 3);              // Отобразим глубину 
  Depth = PressureToDepth(Pressure);
  itoa(Depth, str, 10);
  if (Depth >= 1000 ){
    lcd_putcB(str[0]);
    lcd_putcB(str[1]);
    lcd_putcB('.');
    lcd_putsB(str+2);
  }  
  else if(Depth >= 100){
      lcd_putcB(' ');
      lcd_putcB(str[0]);
      lcd_putcB('.');
      lcd_putsB(str+1);
  }
  else if(Depth >= 10){
      lcd_putcB(' ');
      lcd_putcB('0');
      lcd_putcB('.');
      lcd_putsB(str);
  }
  else{
      lcd_putcB('0');
      lcd_putcB('0');
      lcd_putcB('.');
      lcd_putcB('0');
      lcd_putsB(str);
  }
  lcd_putsB("m ");

  lcd_gotoxy(0, 6);
  //lcd_charMode(NORMALSIZE);   // Отобразим время
  lcd_putc('0' + hour/10);
  lcd_putc('0' + hour%10);
  lcd_putc(':');
  lcd_putc('0' + min/10);
  lcd_putc('0' + min%10);
  lcd_putc(' ');

  //lcd_gotoxy(9, 6);
  itoa(Temp/16, str, 10); // Отобразим температуру
  lcd_puts(str);
  lcd_puts("C ");

}
//---------------------------------------------------------------------
void DrawSurface(void)
{
  
}

//---------------------------------------------------------------------
void Sleep(void)
{
  lcd_gotoxy(0, 4);
  lcd_puts("Sleeping");
	_delay_ms(1000);
	PORTC |= 0x00000001;				// Отключим 5в преобразователь
  lcd_sleep(1);               // Отключим дисплей
 	rtc_write(0x0E, 0b00000000);				// Остановка меандра 1 Гц
  EIMSK &= ~(1<<INT0); // Отключим INT0
  sei();                // На всякий случай лишний раз разрешим прерывания
  Mode = M_AIR;
  SMCR |= 1 << SE | 0 << SM2 | 1 << SM1 | 0 << SM0;      // Разрешим переход в спящий режим. Выберем режим Power Down.
  sleep_mode();
  
	PORTC &= 0x11111110;
  _delay_ms(1000);
  rtc_get_time(&hour, &min, &sec); // Актуализируем время
  EIMSK |= 1<<INT0; // Включим прерывание от часов
 	rtc_write(0x0E, 0b01000000);				// Запустим меандра 1 Гц
  lcd_sleep(0);               // Включим дисплей
	MinPressure = 1200;					// Выход из спящего режима сбрасывает опорное давлениеж
}

//---------------------------------------------------------------------
uint16_t PressureToDepth(uint16_t p)   // Возвращает вычисленную по давлению глубину в сантиметрах
{
  return p - MinPressure;
}
//---------------------------------------------------------------------
uint16_t GetAveragePressure(void)
{
  uint32_t vLongPress = 0;
  for(uint8_t i = 0; i < PRESS_ARRAY_LENGTH; i ++)
  vLongPress += PressArray[i];
  vLongPress /= PRESS_ARRAY_LENGTH;
  return vLongPress;
}
//---------------------------------------------------------------------
