/*
 * DS18B20.h
 *
 * Created: 11.09.2017 13:08:33
 *  Author: sorokineg
 */ 


#ifndef DS18B20_H_
#define DS18B20_H_

#include "global.h"
#include <avr/io.h>
#include <util/delay.h>

#define PORTX PORTD
#define DDRX DDRD
#define PINX PIND
#define DQ 4         // Номер вывода порта на котором сидит 18B20

uint16_t sensor_write(uint8_t cmd);
void sensor_write_bit(uint8_t bit);
uint8_t sensor_read_bit(void);



#endif /* DS18B20_H_ */