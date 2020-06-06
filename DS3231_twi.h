/*
 * DS3231_twi.h
 *
 * Created: 06.06.2020 19:31:40
 *  Author: pac-admin
 */ 


#ifndef DS3231_TWI_H_
#define DS3231_TWI_H_


#define TWEN    2
#define TWIE    0
#define TWINT   7
#define TWEA    6
#define TWSTA   5
#define TWSTO   4
#define TWWC    3

void twi_start(void)
{
	TWCR = (1<<TWEA)|(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
}

void twi_stop(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}

void twi_write(unsigned char _data)
{
	TWDR = _data;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
}

unsigned char twi_read(unsigned char _ack)
{
	unsigned char _data;

	if (_ack==1)
	TWCR = (1<<TWEA)|(1<<TWINT) | (1<<TWEN);
	else
	TWCR = (1<<TWINT) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	_data = TWDR;
	return _data;
}

void rtc_set_time(unsigned char hour,unsigned char min,unsigned char sec)
{
	twi_start();
	twi_write(0xd0);
	twi_write(0);
	twi_write(sec);
	twi_write(min);
	twi_write(hour);
	twi_stop();
}

void rtc_set_date(unsigned char date,unsigned char month,unsigned char year)
{
	twi_start();
	twi_write(0xd0);
	twi_write(4);
	twi_write(date);
	twi_write(month);
	twi_write(year);
	twi_stop();
}

void rtc_get_time(unsigned char *hour,unsigned char *min,unsigned char *sec)
{
	twi_start();
	twi_write(0xd0);
	twi_write(0);
	twi_start();
	twi_write(0xd1);
	*sec=twi_read(1);
	*min=twi_read(1);
	*hour=twi_read(0);
	twi_stop();
}

void rtc_get_date(unsigned char *date,unsigned char *month,unsigned char *year)
{
	twi_start();
	twi_write(0xd0);
	twi_write(4);
	twi_start();
	twi_write(0xd1);
	*date=twi_read(1);
	*month=twi_read(1);
	*year=twi_read(0);
	twi_stop();
}

unsigned char rtc_read(unsigned char address)
{
	unsigned char data;
	twi_start();
	twi_write(0xd0);
	twi_write(address);
	twi_start();
	twi_write(0xd1);
	data=twi_read(0);
	twi_stop();
	return data;
}

void rtc_write(unsigned char address, unsigned char data)
{
	twi_start();
	twi_write(0xd0);
	twi_write(address);
	twi_write(data);
	twi_stop();
}


#endif /* DS3231_TWI_H_ */