/*
 * sdcard.h
 *
 *  Created on: Jun 19, 2021
 *      Author: ACER
 */

#ifndef SDCARD_H_
#define SDCARD_H_
#include <string.h>
#include <stdint.h>


uint8_t SD_Device(char buffer[300],uint8_t port,uint8_t deviceID,uint8_t func,char *deviceChannel,char *deviceType,char *deviceName,char *deviceTitle,char *valueType, char *regtype, uint16_t scale, uint8_t devicestatus);
uint8_t SD_Mqtt(char buffer[200],uint16_t mqtt_port,char *mqttId,char *username,char *pwd, char *broker);
uint8_t SD_Network(char buffer[200],char *ip, char *netmask, char *gateway);
uint8_t SD_Serial(char buffer[200],uint8_t type_serial,uint8_t baud,uint8_t databits, uint8_t stopbit,uint8_t parirty);
uint8_t SD_timeout(char buffer[50],uint16_t timeout);
uint8_t SD_telemetry(char buffer[20], uint8_t telemetry);
uint8_t SD_apikey(char buffer[50],char *apikey);
uint8_t SD_ErrorPacket(char buffer[50],uint8_t port, uint8_t deviceID, uint16_t channel);
uint8_t write_sdcard(char *file,char *buffer);
#endif /* SDCARD_H_ */
