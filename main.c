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

int main(void)
{
  DDRB  = 0b01101111;		//KeyLeft, EnBT, SCK, MISO, MOSI, LCD_CSE, LCD_DC, LCD_RESET
  PORTB = 0b10000000;   // Подтяжка на геркон, BT выключить, ,,,,,,
	
	DDRD = 0b00000000;			// ,,,,,,1s int, 
	
	MCUCR |= (1<<ISC01) | (1<<ISC00); // The rising edge of INT0 generates an interrupt request
	EIMSK |= (1<<INT0); // External Interrupt Request 0 Enable

	//rtc_write(0x02, 0x20);		// Установка часов
	//rtc_write(0x01, 0b01010001); // Установка минут
	//Sec = rtc_read(0x00);
	rtc_write(0x0E, 0b01000000);				// Запуск меандра 1 Гц
	/*if(Sec & 0b10000000)              // Запуск часов
	{
		rtc_write(0x0E, 0b01000000);				//
		rtc_write(0, Sec & 0b01111111);	//
	}
*/

  lcd_init(LCD_DISP_ON);    // init lcd and turn on
	
	lcd_gotoxy(0, 0);
	lcd_putsB(" 10.2M ");
	lcd_gotoxy(0, 3);
	lcd_putsB("  1:40 ");
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
	uint8_t t, Sec_1, Sec_10;

	t = rtc_read(0x00); Sec_1 = t & 0x0F;   Sec_10 = t >> 4;
	t = rtc_read(0x01); Min_1 = t & 0x0F;   Min_10 = t >> 4;
	t = rtc_read(0x02); Hours_1 = t & 0x0F; Hours_10 = t >> 4;
	
	//sei();              // 
	lcd_charMode(DOUBLESIZE);
	lcd_gotoxy(0, 6);
	lcd_putc('0' + Hours_10);
	lcd_putc('0' + Hours_1);
	lcd_putc(':');
	lcd_putc('0' + Min_10);
	lcd_putc('0' + Min_1);
	lcd_putc(' ');
	lcd_putc('1');
	lcd_putc('4');
	lcd_putc('C');
	return;
}
