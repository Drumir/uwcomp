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

void measureBattery(void);
void measurePressure(void);
void measurePressureInSleep(void);
void DrawMenu(void);
void DrawAir(void);
void DrawDeep(void);
void DrawSurface(void);
void Sleep(void);
uint16_t PressureToDepth(uint16_t p);   // ���������� ����������� �� �������� ������� � �����������

char str[10];
//uint8_t ch2 = 0b01010101;
int8_t Mode, CurrentModeLeft, menuItem;
uint8_t hour, min, sec, rtc_temp;
uint8_t contrast;
uint16_t sqw, Vbat, Temp, Depth, Hold;
uint16_t Pressure; // �������� � ������ ��������
uint16_t PressArray[PRESS_ARRAY_LENGTH];
uint8_t PressArrayCount;

int main(void)
{
  DDRB  = 0b01101111;		//KeyRight, BT, SCK, MISO, MOSI, LCD_CSE, LCD_DC, LCD_RESET
  PORTB = 0b11000000;   // �������� �� ������, BT(0-on/1-off), ,,,,,,
	
	DDRD  = 0b10000010;			// BTrst, KeyUp, KeyLeft, DS18B20, INT1(Keys), INT0, TX, RX
	PORTD = 0b01101000;			// 
  
  DDRC  = 0b00000001;   // ,,,,,,,5V
	PORTC = 0b00000000;   // ,,,,,,,5V(0-on/1-off)
  
  ACSR |= 0b10000000; // �������� ���������� ����������
  
 	EICRA |= (1<<ISC01) | (0<<ISC00) | (1<<ISC11) | (0<<ISC10); // The rising edge of INT0 and INT1 generates an interrupt request
	EIMSK |= (1<<INT0) | (1<<INT1); // INT0 Enable, INT1 Enable

  /* ������������� ��� */
  //         ��� En,    not now,  single mode, reset iflag, INTs Disable,       ������������ �������
  ADCSRA = 1 << ADEN | 0 << ADSC | 0 << ADATE | 0 << ADIF | 0 << ADIE | 1 << ADPS2 | 1 << ADPS1 | 1 << ADPS0;
  //	ADCSRA = 0b10001111;		// 1 - ���, 0 - ��� �� �����, 0 - ����������, 0 - ���������� ������������, 0 - ���������� ��� ���������, 111 - ������������ ������� 128
  ADMUX  = 0b11000111;		// 11 - ������� ���������� = 1.1�, 0 - ������������ ������, 0 - ������, 0 - ������, 000 - ����� ������ ADC7
  ADCSRA |= 1<<ADSC;		// ����� �������� ��������� ��������������

  //rtc_set_time(21, 41, 6);
  //rtc_set_date(28, 2, 21);
	rtc_write(0x0E, 0b01000000);				// ������ ������� 1 ��
  
  Mode = M_DEEP;                      // ���� ���������� � ������ ����
  CurrentModeLeft = MENU_MODE_TIME;   // ����� MENU_MODE_TIME ������ ����������� ���� ������ �� ������ M_MENU
  menuItem = MI_TimeH;                 // ���������� ����� ����
  
  PressArrayCount = 0;

  lcd_init(LCD_DISP_ON);              // init lcd and turn on
  contrast = 128;
  lcd_set_contrast(contrast);
  
  rtc_get_time(&hour, &min, &sec);    // ��������� ������� �����
  
  _delay_ms(500);
  EIFR &= (0<<INT1);                  // ������� ���� ���������������� ���������� �� �����
  
	sei();
	
  while (1) 
  {
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

ISR(INT0_vect)                    // ������������ ���������� �� RTC
{
  if(CurrentModeLeft > 0){
    CurrentModeLeft --;
    if (CurrentModeLeft == 0 && Mode == M_MENU)
    {
      Mode = M_AIR;
      CurrentModeLeft = AIR_MODE_TIME;
    }
  }  
  
  sec ++;
  if (sec == 60)
  {
    rtc_get_time(&hour, &min, &sec); // ����� �� RTC ������ ���� ��� � ������ ��� ��������
  }
  
  rtc_temp = rtc_read(0x11); // ������ ����� ����� ����������� RTC
  
	sei();              // 
  
  if(sec%2 == 0)
    sensor_write(0x44);   // ����� ��������� �����������
  else{
    Temp = sensor_write(0xBE); // ������ ������������� ������ c dc18_B_20 / dc18_S_20
  }       
	return;
}
//------------------------------------------------------------------------------

ISR(INT1_vect)                  // ���������� �� ������
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
    }
  }    
	return;
}
//------------------------------------------------------------------------------

ISR(ADC_vect)          // ���������� �������������� ���
{
  MCUCR &= ~(1 << SE);  // ������� ���� ���������� � ������� ������
  uint32_t vLongPress = ADC;
  vLongPress *= 540764;   // ����� �� �������� ���������� �� PC1 ���������� �� ������� �����
  
  //vLongPress /= 100;
  vLongPress /= 11829;
  vLongPress *= 100;
  vLongPress += 34779;
  //PressArray[0] = vLongPress/1000;
  
  PressArray[PressArrayCount] = vLongPress/1000; // �������� �������� � ������ ��������
  PressArrayCount ++; 
  if(PressArrayCount == PRESS_ARRAY_LENGTH) PressArrayCount = 0;
}
//---------------------------------------------------------------------
void measurePressureInSleep(void)    // �������� �������� � KPa
{
  ADMUX  = 0b11000001;		  // 11 - ������� ���������� = 1.1�, 0 - ������������ ������, 0 - ������, 0 - ������, 001 - ����� ������ ADC1
  //         ��� En,    not now,  single mode, reset iflag, INTs Enable,       ������������ �������
  ADCSRA = 1 << ADEN | 0 << ADSC | 0 << ADATE | 0 << ADIF | 1 << ADIE | 1 << ADPS2 | 1 << ADPS1 | 1 << ADPS0;
  MCUCR |= 1 << SE | 0 << SM2 | 0 << SM1 | 1 << SM0;      // �������� ������� � ������ �����. ������� ����� ADC Noise Reduction.
  sleep_mode();
}
//---------------------------------------------------------------------
void measurePressure(void)    // �������� �������� � KPa
{
  //         ��� En,    not now,  single mode, reset iflag, INTs Disable,       ������������ �������
  ADCSRA = 1 << ADEN | 0 << ADSC | 0 << ADATE | 0 << ADIF | 0 << ADIE | 1 << ADPS2 | 1 << ADPS1 | 1 << ADPS0;
  ADMUX  = 0b11000001;		  // 11 - ������� ���������� = 1.1�, 0 - ������������ ������, 0 - ������, 0 - ������, 001 - ����� ������ ADC1
  ADCSRA |= 1<<ADSC;		    // ����� ��������������
  while (ADCSRA & 0x40);		// ���� ����������(������ ����� ADSC � 0)
  uint32_t vLongPress = ADC;
  vLongPress *= 540764;   // ����� �� �������� ���������� �� PC1 ���������� �� ������� �����
  
  //vLongPress /= 100;
  vLongPress /= 11829;
  vLongPress *= 100;
  vLongPress += 34779;
  Pressure = vLongPress/1000;    
}
//------------------------------------------------------------------------------
void measureBattery(void)
{
  //         ��� En,    not now,  single mode, reset iflag, INTs Disable,       ������������ �������
  ADCSRA = 1 << ADEN | 0 << ADSC | 0 << ADATE | 0 << ADIF | 0 << ADIE | 1 << ADPS2 | 1 << ADPS1 | 1 << ADPS0;
  ADMUX  = 0b11000111;		  // 11 - ������� ���������� = 1.1�, 0 - ������������ ������, 0 - ������, 0 - ������, 111 - ����� ������ ADC7
  ADCSRA |= 1<<ADSC;		    // ����� ��������������
  while (ADCSRA & 0x40);		// ���� ����������(������ ����� ADSC � 0)
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
  
}
//---------------------------------------------------------------------
void DrawAir(void)
{
  lcd_gotoxy(2, 3);
  measureBattery();
  itoa(Vbat, str, 10);
  lcd_putsB(str);
  
  lcd_charMode(DOUBLESIZE);
  measurePressure();
  //for(uint8_t i = 0; i < PRESS_ARRAY_LENGTH; i ++)
  //Pressure += PressArray[i];
  //Pressure /= PRESS_ARRAY_LENGTH;
  itoa(Pressure, str, 10);
  lcd_gotoxy(0, 0);
  lcd_putsB(str);
  lcd_putsB("KPa ");
  
  lcd_charMode(DOUBLESIZE);
  lcd_gotoxy(0, 6);
  lcd_putc('0' + hour/10);
  lcd_putc('0' + hour%10);
  lcd_putc(':');
  lcd_putc('0' + min/10);
  lcd_putc('0' + min%10);
  lcd_putc(' ');
  lcd_putc('0' + rtc_temp/10);
  lcd_putc('0' + rtc_temp%10);
  lcd_putc('C');

  itoa(Temp/16, str, 10);
  lcd_gotoxy(0, 3);
  lcd_putsB(str);
  lcd_putsB("C ");

}

//---------------------------------------------------------------------
void DrawDeep(void)
{
  lcd_clrscr();
  
  lcd_gotoxy(0, 0);               // ��������� ��������
  if(Hold < 600) lcd_putcB('0');
  itoa(Hold/60, str, 10);
  lcd_putsB(str);
  lcd_putcB(':');
  if(Hold%60 < 10) lcd_putcB('0');
  itoa(Hold%60, str, 10);
  lcd_putsB(str);

  lcd_gotoxy(0, 3);              // ��������� ������� 
  measurePressureInSleep();
  Pressure = 0;
  for(uint8_t i = 0; i < PRESS_ARRAY_LENGTH; i ++)
    Pressure += PressArray[i];
  Pressure /= PRESS_ARRAY_LENGTH;
  Depth = PressureToDepth(Pressure);
  itoa(Depth, str, 10);
  if (Depth >= 1000 ){
    lcd_putcB(str[0]);
    lcd_putcB(str[1]);
    lcd_putcB(',');
    lcd_putsB(str+2);
  }  
  else if(Depth >= 100){
      lcd_putcB('0');
      lcd_putcB(str[0]);
      lcd_putcB(',');
      lcd_putsB(str+1);
    }
    else {
      lcd_putcB('0');
      lcd_putcB('0');
      lcd_putcB(',');
      lcd_putsB(str);
      }
  lcd_putcB('m');
/*
  measureBattery();
  itoa(Vbat, str, 10);
  lcd_putsB(str);
  
  lcd_charMode(DOUBLESIZE);
  measurePressure();
  itoa(Pressure, str, 10);
  lcd_gotoxy(0, 0);
  lcd_putsB(str);
  lcd_putsB("KPa ");
  
  lcd_charMode(DOUBLESIZE);
  lcd_gotoxy(0, 6);
  lcd_putc('0' + hour/10);
  lcd_putc('0' + hour%10);
  lcd_putc(':');
  lcd_putc('0' + min/10);
  lcd_putc('0' + min%10);
  lcd_putc(' ');
  lcd_putc('0' + rtc_temp/10);
  lcd_putc('0' + rtc_temp%10);
  lcd_putc('C');

  itoa(Temp/16, str, 10);
  lcd_gotoxy(0, 3);
  lcd_putsB(str);
  lcd_putsB("C ");
*/
}
//---------------------------------------------------------------------
void DrawSurface(void)
{
  
}
//---------------------------------------------------------------------
void Sleep(void)
{
  
}

//---------------------------------------------------------------------
uint16_t PressureToDepth(uint16_t p)   // ���������� ����������� �� �������� ������� � �����������
{
  return p - 990;
}
//---------------------------------------------------------------------
