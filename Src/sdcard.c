/*
 * sdcard.c
 *
 *  Created on: Jun 19, 2021
 *      Author: ACER
 */


#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "sdcard.h"

//{"PORT":0,"ID":10,"FC":3,"CHANNEL":"3060","DEVICETYPE":"METER","DEVICENAME":"METER_01","CHANNELTITLE":"ATIVE_POWER_TOTAL","VALUETYPE":"NUMBER","REGTYPE":"FLOAT32","SCALE":1,"DEVICESTATUS":0}
uint8_t SD_Device(char buffer[300],uint8_t port,uint8_t deviceID,uint8_t func,char *deviceChannel,char *deviceType,char *deviceName,char *deviceTitle,char *valueType, char *regtype, uint16_t scale, uint8_t devicestatus)
{
    memset(buffer,'\0',sizeof(buffer));
    sprintf(buffer,"{\"PORT\":%d,\"ID\":%d,\"FC\":%d,\"CHANNEL\":\"%s\",\"DEVICETYPE\":\"%s\",\"DEVICENAME\":\"%s\",\"CHANNELTITLE\":\"%s\",\"VALUETYPE\":\"%s\",\"REGTYPE\":\"%s\",\"SCALE\":%d,\"DEVICESTATUS\":%d}\n",port,deviceID,func,deviceChannel,deviceType,deviceName,deviceTitle,valueType,regtype,scale,devicestatus);
}
//{"network":{"ip":"192.168.100.111","netmask":"255.255.255.0","gateway":"192.168.100.1","broker":"95.111.195.76"}}
uint8_t SD_Network(char buffer[200],char *ip, char *netmask, char *gateway)
{
    memset(buffer,'\0',200);
    sprintf(buffer,"{\"network\":{\"ip\":\"%s\",\"netmask\":\"%s\",\"gateway\":\"%s\"}}\n",ip, netmask, gateway);
}
/* {"mqttId":"null","username":"null","pwd":"null","port":1883,"apikey":"60cda6bc55193093bbcd001f"}*/
uint8_t SD_Mqtt(char buffer[200],uint16_t mqtt_port,char *mqttId,char *username,char *pwd, char *broker)
{
    memset(buffer,'\0',200);
    sprintf(buffer,"{\"mqtt\":{\"broker\":\"%s\",\"mqttId\":\"%s\",\"username\":\"%s\",\"pwd\":\"%s\",\"port\":%d}}\n", broker, mqttId,username,pwd,mqtt_port);
}
/*
 * baud = 1200xhs(4800, 9600, 19200, 115200)*/
/* {"rs232":{"baud":115200,"databits":8,"stopbits":1,"parity":0}} */
uint8_t SD_Serial(char buffer[100],uint8_t type_serial,uint8_t baud,uint8_t databits, uint8_t stopbit,uint8_t parirty)
{
    memset(buffer,'\0',sizeof(buffer));
    if (type_serial==1){ //rs232
    	 sprintf(buffer,"{\"rs232\":{\"baud\":%d,\"databits\":%d,\"stopbits\":%d,\"parity\":%d}}\n",baud,databits,stopbit,parirty);
    }else if(type_serial==2){ //port 0
    	sprintf(buffer,"{\"port0\":{\"baud\":%d,\"databits\":%d,\"stopbits\":%d,\"parity\":%d}}\n",baud,databits,stopbit,parirty);
    }else if(type_serial==3){ //port 1
    	sprintf(buffer,"{\"port1\":{\"baud\":%d,\"databits\":%d,\"stopbits\":%d,\"parity\":%d}}\n",baud,databits,stopbit,parirty);
    }
}
uint8_t SD_timeout(char buffer[50],uint16_t timeout)
{
    memset(buffer,'\0',sizeof(buffer));
    sprintf(buffer,"{\"timeout\":%d}\n",timeout);
}
uint8_t SD_apikey(char buffer[50],char *apikey)
{
    memset(buffer,'\0',sizeof(buffer));
    sprintf(buffer,"{\"apikey\":\"%s\"}\n",apikey);
}
uint8_t SD_telemetry(char buffer[20], uint8_t telemetry)
{
    sprintf(buffer,"{\"telemetry\":%d}\n",telemetry);
}

uint8_t SD_ErrorPacket(char buffer[50],uint8_t port, uint8_t deviceID, uint16_t channel)
{
	memset(buffer,'\0',sizeof(buffer));
	uint8_t time[6];
	char minute[3];
	char second[3];
	getTime(time);
	if (time[4] < 10){
		 sprintf(minute,"0%d",time[4]);
	}else
		 sprintf(minute,"%d",time[4]);
	if (time[5] < 10){
		 sprintf(second,"0%d",time[5]);
	}else
		 sprintf(second,"%d",time[5]);
    sprintf(buffer,"{\"id\":%d,\"channel\":%d,\"time\":\"20%d-0%d-0%dT%d:%s:%s.000Z\"}\n", deviceID, channel,time[0],time[1],time[2],time[3],minute,second);
}
