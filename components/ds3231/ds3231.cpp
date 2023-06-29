/*
 * DS3231.cpp
 *
 *  Created on: Nov 29, 2022
 *      Author: anh
 */

#include "component_config.h"
#if ENABLE_COMPONENT_DS3231

#include "periph/ds3231.h"



static i2c_t _i2c;
static uint8_t decToBcd(int val){
  return (uint8_t)((val/10*16) + (val%10));
}

static int bcdToDec(uint8_t val){
  return (int)((val/16*10) + (val%16));
}

static void DS3231_Temp_Conv(void){
	uint8_t status = 0;
	uint8_t control = 0;
	_i2c -> i2c_read_register(DS3231_ADDRESS, 0x0F, 1, &status, 1);
	if (!(status & 0x04)){
		_i2c -> i2c_read_register(DS3231_ADDRESS, 0x0E, 1, &control, 1);
		_i2c -> i2c_write_register(DS3231_ADDRESS, 0x0E, 1, (uint8_t *)(control|(0x20)), 1);
	}
}

void DS3231_Init(i2c_t i2c){
	_i2c = i2c;

	DS3231_Temp_Conv();
}

void DS3231_SetTime(DS3231_Time_t time){
	uint8_t set_time[7];
	set_time[0] = decToBcd(time.seconds );
	set_time[1] = decToBcd(time.minutes);
	set_time[2] = decToBcd(time.hour);
	set_time[3] = decToBcd(time.dayofweek);
	set_time[4] = decToBcd(time.dayofmonth);
	set_time[5] = decToBcd(time.month);
	set_time[6] = decToBcd(time.year);

	_i2c -> i2c_write_register(DS3231_ADDRESS, 0x00, 1, set_time, 7);
}

void DS3231_GetTime(DS3231_Time_t *time){
	uint8_t get_time[7];

	_i2c -> i2c_read_register(DS3231_ADDRESS, 0x00, 1, get_time, 7);

	time->seconds 	 = bcdToDec(get_time[0]);
	time->minutes 	 = bcdToDec(get_time[1]);
	time->hour 		 = bcdToDec(get_time[2]);
	time->dayofweek  = bcdToDec(get_time[3]);
	time->dayofmonth = bcdToDec(get_time[4]);
	time->month 	 = bcdToDec(get_time[5]);
	time->year       = bcdToDec(get_time[6]);
}

float DS3231_GetTemp(void){
	uint8_t temp[2];

	_i2c -> i2c_read_register(DS3231_ADDRESS, 0x11, 1, temp, 2);
	return (float)(temp[0] + (temp[1]>>6)/4.0);
}

#endif /* ENABLE_COMPONENT_SPIFLASH */

