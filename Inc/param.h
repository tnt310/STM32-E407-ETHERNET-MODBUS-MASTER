/*
 * param.h
 *
 *  Created on: Mar 31, 2021
 *      Author: ACER
 */

#ifndef PARAM_H_
#define PARAM_H_
#include "main.h"

typedef struct
{
	uint8_t channel;
	uint8_t id;
	uint8_t func;
	uint8_t numreg;
	uint16_t regAdr;
	uint8_t status;
}device_t;

typedef struct
{
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint8_t day;
	uint8_t month;
	uint8_t year;
}Time_t;



//extern const char *tel_topic;
//extern const char *command;
//extern const char *down_pro;
//extern const char *pro_topic;

#endif /* PARAM_H_ */
