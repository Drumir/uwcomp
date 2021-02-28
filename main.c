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
#include "util/delay.h"
#include "uwcomp.h"
#include "font.h"
#include "lcd.h"
#include "DS3231_twi.h"
#include "DS18B20.h"

void measureBattery(void);
void measurePressure(void);



char str[10];
uint8_t ch2 = 0b01010101;
uint8_t Sec, Min_1, Min_10, Hours_1, Hours_10;
uint8_t hour, min, sec;
uint16_t sqw, Vbat, Pressure;

int main(void)
{
  DDRB  = 0b01101111;		//KeyLeft, EnBT, SCK, MISO, MOSI, LCD_CSE, LCD_DC, LCD_RESET
  PORTB = 0b10000000;   // Подтяжка на геркон, BT выключить, ,,,,,,
	
	DDRD = 0b00000000;			// ,,,,,,1s int, 
  
  DDRC  = 0b00000001;   // ,,,,,,,Enable 5V
	PORTC = 0b00000001;   //
  
	EICRA |= (1<<ISC01) | (1<<ISC00); // The rising edge of INT0 generates an interrupt request
	EIMSK |= (1<<INT0); // External Interrupt Request 0 Enable

  /* Инициализация АЦП */
  //         АЦП En,    not now,  single mode, reset iflag, INTs Disable,       предделитель частоты
  ADCSRA = 1 << ADEN | 0 << ADSC | 0 << ADATE | 0 << ADIF | 0 << ADIE | 1 << ADPS2 | 1 << ADPS1 | 1 << ADPS0;
  //	ADCSRA = 0b10001111;		// 1 - вкл, 0 - еще не старт, 0 - однократно, 0 - прерывания генерировать, 0 - прерывания АЦП разрешить, 111 - предделитель частоты 128
  ADMUX  = 0b11000111;		// 11 - Опорное напряжение = 1.1В, 0 - выравнивание вправо, 0 - резерв, 0 - резерв, 000 - выбор канала ADC7
  ADCSRA |= 1<<ADSC;		// Старт пробного мусорного преобразования

  rtc_set_time(0, 45, 6);
  //rtc_set_date(28, 2, 21);
	rtc_write(0x0E, 0b01000000);				// Запуск меандра 1 Гц
  
  Vbat = 1111;

  lcd_init(LCD_DISP_ON);    // init lcd and turn on
	/*
	lcd_gotoxy(0, 0);
	lcd_putsB(" 4.21m ");
	lcd_gotoxy(0, 3);
	lcd_putsB("  0:41 ");
  */
  //rtc_get_time(&hour, &min, &sec);
  
	sei();
	
  /* Replace with your application code */
//  uint8_t contrast = 0;
  while (1) 
  {
    sensor_write(0x44);   // старт измерения температуры
    _delay_ms(1000);
    uint16_t Temp = sensor_write(0xBE); // чтение температурных данных c dc18_B_20 / dc18_S_20
    itoa(Temp/16, str, 10);
  	lcd_gotoxy(0, 3);
//	  lcd_putsB("");
	  lcd_putsB(str);
	  lcd_putsB("C ");
    measureBattery();
    itoa(Vbat, str, 10);
    lcd_putsB(str);
    //lcd_putsB(" ");
    measurePressure();
    itoa(Pressure, str, 10);
    lcd_gotoxy(0, 0);
    lcd_putsB(str);
    lcd_putsB("KPa ");
    


    /*
    lcd_set_contrast(contrast);
    itoa((int)contrast, str, 10);
    strcat(str, "   ");
    lcd_gotoxy(0,2);          // set cursor to first column at line 3
    lcd_puts(str);  // puts string form flash to display (TEXTMODE) or buffer (GRAPHICMODE)
    _delay_ms(100);
    contrast ++;*/
  }
}

//--------------------------------------------------------------
//10.8 28.3C
//1:45
ISR(INT0_vect) // 
{
  rtc_get_time(&hour, &min, &sec);
  
  uint8_t t = rtc_read(0x11); // Чтение целой части температуры RTC
  
	//sei();              // 
	lcd_charMode(DOUBLESIZE);
	lcd_gotoxy(0, 6);
	lcd_putc('0' + hour/10);
	lcd_putc('0' + hour%10);
  lcd_putc(':');
	lcd_putc('0' + min/10);
	lcd_putc('0' + min%10);
	lcd_putc(' ');
	lcd_putc('0' + t/10);
	lcd_putc('0' + t%10);
	lcd_putc('C');
	return;
}
//------------------------------------------------------------------------------
ISR(ADC_vect)          // Завершение преобразования АЦП
{
  //Vbat = ADC;		// Преобразуем условные единицы в вольты
}
//------------------------------------------------------------------------------
void measureBattery(void)
{
  ADMUX  = 0b11000111;		  // 11 - Опорное напряжение = 1.1В, 0 - выравнивание вправо, 0 - резерв, 0 - резерв, 111 - выбор канала ADC7
  ADCSRA |= 1<<ADSC;		    // Старт преобразования
  while (ADCSRA & 0x40);		// Ждем завершения(сброса флага ADSC в 0)
  uint32_t vLongBat = ADC;
  vLongBat *= 1000;
  Vbat = vLongBat / 2291;
}
//---------------------------------------------------------------------
void measurePressure(void)
{
  ADMUX  = 0b11000001;		  // 11 - Опорное напряжение = 1.1В, 0 - выравнивание вправо, 0 - резерв, 0 - резерв, 001 - выбор канала ADC1
  ADCSRA |= 1<<ADSC;		    // Старт преобразования
  while (ADCSRA & 0x40);		// Ждем завершения(сброса флага ADSC в 0)
  uint32_t vLongPress = ADC;
  vLongPress *= 540764;   // Здесь мы получаем напряжение на PC1 умноженное на миллион вроде
  vLongPress /= 100;
  vLongPress /= 11829;
  vLongPress *= 10000;
  vLongPress += 34779;
  Pressure = vLongPress/10000;
}
//---------------------------------------------------------------------
