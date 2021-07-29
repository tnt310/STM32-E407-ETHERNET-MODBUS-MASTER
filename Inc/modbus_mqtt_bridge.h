#ifndef __MODBUS_MQTT_BRIDGE__H__
#define __MODBUS_MQTT_BRIDGE__H__
//#include "MQTTClient.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"
#include "lwip/err.h"
#include "param.h"
#include <stdint.h>
/* Public define*/
#define JSON_MAX_LEN 200
#define portDEFAULT_WAIT_TIME 1000
#define MAX_JSON_LEN 		  6000
#define TEMP_JSON_LEN 		  50

//#define ""
extern char recordbuffer[1000];

/* Public function prototype ----------------*/
uint8_t mqtt_modbus_thread_down_provision(char *Buffer,uint16_t BufferLen);
void mqtt_modbus_thread_up(mqtt_client_t *client, char *pub_topic, char* pro_topic, char* command_topic, char *time_topic);

uint8_t mqtt_modbus_thread_down_command(char *pJsonMQTTBuffer,uint16_t pJsonMQTTBufferLen);

uint8_t topic(char *buffer, char *flow, char *topic, char *api_key);

data1_t *parse_device(char *Buffer, uint16_t BufferLen);
void parse_sdcardInfo(char *Buffer, uint16_t BufferLen);
uint8_t parse_device_provision(char *Buffer, uint16_t BufferLen, uint8_t deviceID, uint16_t channelID, uint8_t channel_status);
void parse_mqttInfor(char *Buffer, uint16_t BufferLen);
void addDevice(data1_t *destination, data1_t *data);
void LoadDevice(void);
#endif
