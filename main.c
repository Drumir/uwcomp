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
void DrawMenu(void);
void DrawAir(void);

char str[10];
//uint8_t ch2 = 0b01010101;
int8_t Mode, CurrentModeLeft, menuItem;
uint8_t hour, min, sec, rtc_temp;
uint8_t contrast;
uint16_t sqw, Vbat, Pressure, Temp;

int main(void)
{
  DDRB  = 0b01101111;		//KeyRight, BT, SCK, MISO, MOSI, LCD_CSE, LCD_DC, LCD_RESET
  PORTB = 0b11000000;   // �������� �� ������, BT(0-on/1-off), ,,,,,,
	
	DDRD  = 0b10000010;			// BTrst, KeyUp, KeyLeft, DS18B20, INT1(Keys), INT0, TX, RX
	PORTD = 0b01101000;			// 
  
  DDRC  = 0b00000001;   // ,,,,,,,5V
	PORTC = 0b00000000;   //,,,,,,,5v(0-on/1-off)
  
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
  
  Mode = M_MENU;                      // ���� ���������� � ������ ����
  CurrentModeLeft = MENU_MODE_TIME;   // ����� MENU_MODE_TIME ������ ����������� ���� ������ �� ������ M_MENU
  menuItem = MI_TimeH;                 // ���������� ����� ����

  lcd_init(LCD_DISP_ON);              // init lcd and turn on
  contrast = 128;
  lcd_set_contrast(contrast);
  
  rtc_get_time(&hour, &min, &sec);    // ��������� ������� �����
  
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
  //Vbat = ADC;		// ����������� �������� ������� � ������
}
//------------------------------------------------------------------------------
void measureBattery(void)
{
  ADMUX  = 0b11000111;		  // 11 - ������� ���������� = 1.1�, 0 - ������������ ������, 0 - ������, 0 - ������, 111 - ����� ������ ADC7
  ADCSRA |= 1<<ADSC;		    // ����� ��������������
  while (ADCSRA & 0x40);		// ���� ����������(������ ����� ADSC � 0)
  uint32_t vLongBat = ADC;
  vLongBat *= 1000;
  Vbat = vLongBat / 2291;
}
//---------------------------------------------------------------------
void measurePressure(void)
{
  ADMUX  = 0b11000001;		  // 11 - ������� ���������� = 1.1�, 0 - ������������ ������, 0 - ������, 0 - ������, 001 - ����� ������ ADC1
  ADCSRA |= 1<<ADSC;		    // ����� ��������������
  while (ADCSRA & 0x40);		// ���� ����������(������ ����� ADSC � 0)
  uint32_t vLongPress = ADC;
  vLongPress *= 540764;   // ����� �� �������� ���������� �� PC1 ���������� �� ������� �����
  
  vLongPress /= 100;
  vLongPress /= 11829;
  vLongPress *= 10000;
  vLongPress += 34779;
  Pressure = vLongPress/10000;
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
