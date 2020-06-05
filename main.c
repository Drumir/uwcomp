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
#include "util/delay.h"
#include "uwcomp.h"
#include "font.h"
#include "lcd.h"

char str[10];

int main(void)
{
  DDRB  = 0b01101111;		//KeyLeft, EnBT, SCK, MISO, MOSI, LCD_CSE, LCD_DC, LCD_RESET
  PORTB = 0b10000000;   // Подтяжка на геркон, BT выключить, ,,,,,,

  lcd_init(LCD_DISP_ON);    // init lcd and turn on
  //lcd_charMode(DOUBLESIZE);
  for(uint8_t x = 0, y = 0; y < 8; x ++, y ++)
  {
    lcd_gotoxy(x,y);
    lcd_puts("Hello World");
  }
  /* Replace with your application code */
  uint8_t contrast = 0;
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
