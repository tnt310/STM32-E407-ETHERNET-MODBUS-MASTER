/*
 * time_value.h
 *
 *  Created on: Apr 14, 2021
 *      Author: ACER
 */

#ifndef TIME_VALUE_H_
#define TIME_VALUE_H_

/* Built-in C library includes ---------------*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Platform includes --------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "main.h"
#include "fs.h"
#include "rtc.h"
#include "flash.h"
/* Shared Variable ----------------------------------*/
#include "lwip.h"
#include "socket.h"
/* Includes ------------------------------------------------------------------*/
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/apps/fs.h"
#include "tcp.h"
#include "string.h"
#include "cmsis_os.h"
#include <stdio.h>

uint8_t getTime(uint8_t time[6]);

#endif /* TIME_VALUE_H_ */
