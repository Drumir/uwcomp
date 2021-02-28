/*
 * UWComp.c
 *
 * Created: 27.05.2020 23:27:06
 * Author : Drumir
 */ 
#define F_CPU 8000000


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

char str[10];
uint8_t ch2 = 0b01010101;
uint8_t Sec, Min_1, Min_10, Hours_1, Hours_10;
uint8_t hour, min, sec;
uint16_t sqw;

int main(void)
{
  DDRB  = 0b01101111;		//KeyLeft, EnBT, SCK, MISO, MOSI, LCD_CSE, LCD_DC, LCD_RESET
  PORTB = 0b10000000;   // Подтяжка на геркон, BT выключить, ,,,,,,
	
	DDRD = 0b00000000;			// ,,,,,,1s int, 
	
	EICRA |= (1<<ISC01) | (1<<ISC00); // The rising edge of INT0 generates an interrupt request

	EIMSK |= (1<<INT0); // External Interrupt Request 0 Enable

	//rtc_write(0x00, 0b00100010);				// Установка секунд
	//rtc_write(0x01, 0b00000010);				// Установка минут
	//rtc_write(0x02, 0b00000000);				// Установка часов
  //rtc_set_time(19, 10, 10);
  rtc_set_date(28, 2, 21);
	rtc_write(0x0E, 0b01000000);				// Запуск меандра 1 Гц

  lcd_init(LCD_DISP_ON);    // init lcd and turn on
	
	lcd_gotoxy(0, 0);
	lcd_putsB(" 4.21m ");
	lcd_gotoxy(0, 3);
	lcd_putsB("  0:41 ");
  
  //rtc_get_time(&hour, &min, &sec);
  
	sei();
	
  /* Replace with your application code */
//  uint8_t contrast = 0;
  while (1) 
  {/*
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
  
  uint8_t t = rtc_read(0x11); // Чтение температуры RTC
  
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
  t = rtc_read(0x03); // Чтение дня недели
	lcd_putc('0' + t);
	return;
}
