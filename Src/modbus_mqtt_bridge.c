/* Built-in C library includes ---------------*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <param.h>
#include <stdlib.h>
/* Platform includes --------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "main.h"
#include "mqttclienttask.h"
#include "flash.h"
#include "rtc.h"
#include "usart.h"
#include "json.h"
#include "modbus_mqtt_bridge.h"
#include "mbproto.h"

/* Network includes----------------------------*/
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"
#include "lwip/ip4_addr.h"
#include "err.h"
#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"

#include "command.h"
#include "http_client.h"
#include "param.h"
#include "time_value.h"
#include "telemetry.h"


/* Shared Variables --------------------------*/
extern osMessageQId xQueueDownlinkHandle;
extern osMessageQId xQueueUplinkHandle;
network_param_t netParam;
network_param_t mqttHostParam;
uint32_t 		mqtt_port;
uint32_t 		timeDelay; // timeout: 15s, 30s, 1p, 3p, 5p, 10p
char *mqtt_id;
char *mqtt_user;
char *mqtt_password;
char *apikey;
uint16_t u16_mqtt_port;
/* Private Variables -------------------------*/
uint8_t mqtt_couter_err = 0;
char buffer[100];
char temp[20];
char temp0[10];
char temp1[20];
char temp2[20];
char temp3[20];
char ip_temp[20];
char netmask_temp[20];
char gateway_temp[20];
char broker_temp[20];
char Id[20];
char user[20];
char passwork[20];
char key[30];
uint32_t port0_baud,port0_stop,port0_databit,port0_parity;
uint32_t port1_baud,port1_stop,port1_databit,port1_parity;

uint8_t num_device;
static data1_t *ptr;
data1_t test;

data1_t *dynamic;
static uint8_t counter = 0;
static uint8_t telemetry = 0;
uint8_t id_temp[2];
uint8_t num_device;
/* Start implementation ----------------------*/

char *allocate(char *src)
{
	char *dst = pvPortMalloc(strlen(src)+1);
	strcpy(dst, src);
	return dst;
	}

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
	if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start && strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
		return 0;
	}
	return -1;
}
/*-----------------------------------------PARSING DATA FROM SDCARD-----------------------------------------------------------------------------*/
void parse_sdcardInfo(char *Buffer, uint16_t BufferLen)
{
	int r;
	jsmn_parser p;
	jsmntok_t t[JSON_MAX_LEN]; /* We expect no more than JSON_MAX_LEN tokens */
	jsmn_init(&p);
	r = jsmn_parse(&p, Buffer,BufferLen, t,sizeof(t) / sizeof(t[0]));
	for (uint8_t i = 0; i < r; i++){
		if (jsoneq(Buffer, &t[i], "network") == 0){
			//printf("\r\n - network: %.*s\r\n", t[i + 1].end - t[i + 1].start,Buffer + t[i + 1].start);
			for (uint8_t j = i; j < r-1; j++){
				if (jsoneq(Buffer, &t[j], "ip") == 0){
					//printf("\r\n -ip: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					//char ip_temp[20];
					strncpy(ip_temp,Buffer + t[j + 1].start, t[j + 1].end - t[j + 1].start);
					ip4_addr_t ip;
					if (ipaddr_aton(ip_temp, &ip)) {
						netParam.ip.idata = ip.addr;
						printf("\r\n New IP: %d %d %d %d", netParam.ip.cdata[0],netParam.ip.cdata[1], netParam.ip.cdata[2],netParam.ip.cdata[3]);
					}
					j++;
				}else if (jsoneq(Buffer, &t[j], "netmask") == 0){
					//printf("\r\n -netmask: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					//char netmask_temp[20];
					strncpy(netmask_temp,Buffer + t[j + 1].start, t[j + 1].end - t[j + 1].start);
					ip4_addr_t ip;
					if (ipaddr_aton(netmask_temp, &ip)) {
						netParam.netmask.idata = ip.addr;
						printf("\r\n New Netmask: %d %d %d %d", netParam.netmask.cdata[0],netParam.netmask.cdata[1], netParam.netmask.cdata[2],netParam.netmask.cdata[3]);
					}
					j++;
				}else if (jsoneq(Buffer, &t[j], "gateway") == 0){
					//printf("\r\n -gateway: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					//char gateway_temp[20];
					strncpy(gateway_temp,Buffer + t[j + 1].start, t[j + 1].end - t[j + 1].start);
					ip4_addr_t ip;
					if (ipaddr_aton(gateway_temp, &ip)) {
						netParam.gateway.idata = ip.addr;
						printf("\r\n New gateway: %d %d %d %d", netParam.gateway.cdata[0],netParam.gateway.cdata[1], netParam.gateway.cdata[2],netParam.gateway.cdata[3]);
					}
					j++;
				}else if (jsoneq(Buffer, &t[j], "broker") == 0){
					//printf("\r\n -broker: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					//char broker_temp[20];
					strncpy(broker_temp,Buffer + t[j + 1].start, t[j + 1].end - t[j + 1].start);
					ip4_addr_t ip;
					if (ipaddr_aton(broker_temp, &ip)) {
						mqttHostParam.ip.idata = ip.addr;
						printf("\r\n New Broker IP: %d %d %d %d", mqttHostParam.ip.cdata[0],mqttHostParam.ip.cdata[1], mqttHostParam.ip.cdata[2],mqttHostParam.ip.cdata[3]);
					}
					j++;
				}
			}
			break;
		}else if (jsoneq(Buffer, &t[i], "mqtt") == 0){
			printf("\r\n - mqtt: %.*s\r\n", t[i + 1].end - t[i + 1].start,Buffer + t[i + 1].start);
			for (uint8_t j = i; j < r-1; j++){
				if (jsoneq(Buffer, &t[j], "mqttId") == 0){
					printf("\r\n -mqttId: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					//char Id[20];
					strncpy(Id,Buffer + t[j + 1].start, t[j + 1].end - t[j + 1].start);
					mqtt_id = Id;
					j++;
				}else if (jsoneq(Buffer, &t[j], "username") == 0){
					printf("\r\n -username: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					//char user[20];
					strncpy(user,Buffer + t[j + 1].start, t[j + 1].end - t[j + 1].start);
					mqtt_user = user;
					j++;
				}else if (jsoneq(Buffer, &t[j], "pwd") == 0){
					printf("\r\n -pwd: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					//char passwork[20];
					strncpy(passwork,Buffer + t[j + 1].start, t[j + 1].end - t[j + 1].start);
					mqtt_password = passwork;
					j++;
				}else if (jsoneq(Buffer, &t[j], "port") == 0){
					printf("\r\n -port: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					u16_mqtt_port= atoi(Buffer + t[j + 1].start);
					j++;
				}else if (jsoneq(Buffer, &t[j], "apikey") == 0){
					printf("\r\n -apikey: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					//char key[30];
					strncpy(key,Buffer + t[j + 1].start, t[j + 1].end - t[j + 1].start);
					apikey = key;
					j++;
				}
			}
			break;
		}else if (jsoneq(Buffer, &t[i], "rs232") == 0){
			printf("\r\n - rs232: %.*s\r\n", t[i + 1].end - t[i + 1].start,Buffer + t[i + 1].start);
			for (uint8_t j = i; j < r-1; j++){
				if (jsoneq(Buffer, &t[j], "baud") == 0){
					printf("\r\n -baud: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					j++;
				}else if (jsoneq(Buffer, &t[j], "databits") == 0){
					printf("\r\n -databits: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					j++;
				}else if (jsoneq(Buffer, &t[j], "stopbits") == 0){
					printf("\r\n -stopbits: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					j++;
				}else if (jsoneq(Buffer, &t[j], "parity") == 0){
					printf("\r\n -parity: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					j++;
				}
			}
			break;
		}else if (jsoneq(Buffer, &t[i], "port0") == 0){
			printf("\r\n - port0: %.*s\r\n", t[i + 1].end - t[i + 1].start,Buffer + t[i + 1].start);
			for (uint8_t j = i; j < r-1; j++){
				if (jsoneq(Buffer, &t[j], "baud") == 0){
					printf("\r\n -baud: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					port0_baud = (uint32_t)atoi(Buffer + t[j + 1].start);
					j++;
				}else if (jsoneq(Buffer, &t[j], "databits") == 0){
					printf("\r\n -databits: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					port0_databit = (uint32_t)atoi(Buffer + t[j + 1].start);
					j++;
				}else if (jsoneq(Buffer, &t[j], "stopbits") == 0){
					printf("\r\n -stopbits: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					port0_stop = (uint32_t)atoi(Buffer + t[j + 1].start);
					j++;
				}else if (jsoneq(Buffer, &t[j], "parity") == 0){
					printf("\r\n -parity: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					port0_parity = (uint32_t)atoi(Buffer + t[j + 1].start);
					j++;
				}
			}
			break;
		}else if (jsoneq(Buffer, &t[i], "port1") == 0){
			printf("\r\n - port1: %.*s\r\n", t[i + 1].end - t[i + 1].start,Buffer + t[i + 1].start);
			for (uint8_t j = i; j < r-1; j++){
				if (jsoneq(Buffer, &t[j], "baud") == 0){
					printf("\r\n -baud: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					port1_baud = (uint32_t)atoi(Buffer + t[j + 1].start);
					j++;
				}else if (jsoneq(Buffer, &t[j], "databits") == 0){
					printf("\r\n -databits: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					port1_databit = (uint32_t)atoi(Buffer + t[j + 1].start);
					j++;
				}else if (jsoneq(Buffer, &t[j], "stopbits") == 0){
					printf("\r\n -stopbits: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					port1_stop = (uint32_t)atoi(Buffer + t[j + 1].start);
					j++;
				}else if (jsoneq(Buffer, &t[j], "parity") == 0){
					printf("\r\n -parity: %.*s\r\n", t[j + 1].end - t[j + 1].start,Buffer + t[j + 1].start);
					port1_parity = (uint32_t)atoi(Buffer + t[j + 1].start);
					j++;
				}
			}
			break;
		}else if (jsoneq(Buffer, &t[i], "timeout") == 0){
			printf("\r\n - timeout: %.*s\r\n", t[i + 1].end - t[i + 1].start,Buffer + t[i + 1].start);
			timeDelay = atoi(Buffer + t[i + 1].start);
			break;
		}
	}
}

data1_t *parse_device(char *Buffer, uint16_t BufferLen)
{
	ptr = &test;
	int r;
	jsmn_parser p;
	jsmntok_t t[JSON_MAX_LEN]; /* We expect no more than JSON_MAX_LEN tokens */
	jsmn_init(&p);
	r = jsmn_parse(&p, Buffer,BufferLen, t,sizeof(t) / sizeof(t[0]));
	for (uint8_t i = 1; i < r; i++) {
		if (jsoneq(Buffer, &t[i], "PORT") == 0) {
			ptr->channel = (uint8_t)atoi(Buffer + t[i + 1].start);
			i++;
		} else if (jsoneq(Buffer, &t[i], "ID") == 0) {
			ptr->deviceID = (uint8_t)atoi(Buffer + t[i + 1].start);
			i++;
		} else if (jsoneq(Buffer, &t[i], "FC") == 0) {
			ptr->func = (uint8_t)atoi(Buffer + t[i + 1].start);
			i++;
		} else if (jsoneq(Buffer, &t[i], "CHANNEL") == 0) {
			memset(temp0,'\0',sizeof(temp0));
			strncpy(temp0,Buffer + t[i + 1].start, t[i + 1].end - t[i + 1].start);
			ptr->deviceChannel = (uint16_t)strtol(temp0, NULL, 0);
			i++;
		} else if (jsoneq(Buffer, &t[i], "DEVICETYPE") == 0) {
			memset(temp,'\0',sizeof(temp));
			strncpy(temp,Buffer + t[i + 1].start, t[i + 1].end - t[i + 1].start);
			ptr->deviceType = temp;
			i++;
		} else if (jsoneq(Buffer, &t[i], "DEVICENAME") == 0) {
			memset(temp1,'\0',sizeof(temp));
			strncpy(temp1,Buffer + t[i + 1].start, t[i + 1].end - t[i + 1].start);
			ptr->deviceName = temp1;
			i++;
		} else if (jsoneq(Buffer, &t[i], "CHANNELTITLE") == 0) {
			memset(temp2,'\0',sizeof(temp2));
			strncpy(temp2,Buffer + t[i + 1].start, t[i + 1].end - t[i + 1].start);
			ptr->channeltitle = temp2;
			i++;
		}else if (jsoneq(Buffer, &t[i], "VALUETYPE") == 0) {
			memset(temp3,'\0',sizeof(temp3));
			strncpy(temp3,Buffer + t[i + 1].start, t[i + 1].end - t[i + 1].start);
			ptr->valueType= temp3;
			i++;
		}else if (jsoneq(Buffer, &t[i], "NUMREG") == 0) {
			ptr->numreg = (uint8_t)atoi(Buffer + t[i + 1].start);
			i++;
		}else if (jsoneq(Buffer, &t[i], "SCALE") == 0) {
			ptr->scale = (uint8_t)atoi(Buffer + t[i + 1].start);
			i++;
		} else if (jsoneq(Buffer, &t[i], "DEVICESTATUS") == 0) {
			ptr->devicestatus = atoi(Buffer + t[i + 1].start);
			i++;
		}
	}
	return ptr;
}

void addDevice(data1_t *destination, data1_t *data)
{
   destination->channel = data->channel;
   destination->deviceID = data->deviceID;
   destination->func = data->func;
   destination->devicestatus = data->devicestatus;
   destination->numreg = data->numreg;
   destination->scale = data->scale;
   destination->deviceChannel = data->deviceChannel;
   destination->deviceType = allocate(data->deviceType);
   destination->deviceName = allocate(data->deviceName);
   destination->channeltitle = allocate(data->channeltitle);
   destination->valueType = allocate(data->valueType);
}

/*---------------------------------------------------MODBUS MQTT BRIGDE FUNCTION-----------------------------------------------------------------------------*/
static void mqtt_bridge_pub_request_cb(void *arg, err_t result) {
	if (result != ERR_OK) {
		printf("\r\n Publish telemetry result: %d\n", result);
	} else {
	}
}
static void mqtt_bridge_provision_request_cb(void *arg, err_t result) {
	if (result != ERR_OK) {
		printf("\r\n Publish provision result: %d\n", result);
	} else {
	}
}
static void mqtt_bridge_command_request_cb(void *arg, err_t result) {
	if (result != ERR_OK) {
		printf("\r\n Publish command result: %d\n", result);
	} else {
	}
}
/*----------------------------------------------------UP/DOWNLINK FUNCTION--------------------------------------------------------------------------*/
void mqtt_modbus_thread_up(mqtt_client_t *client, char *pub_topic, char* pro_topic, char* command_topic) {

	uint8_t time[6];
	BaseType_t Err = pdFALSE;
	xQueueMbMqtt_t xQueueMbMqtt;
	portCHAR head[MAX_JSON_LEN];
	portCHAR tail[MAX_JSON_LEN];
	portCHAR jsontemp[TEMP_JSON_LEN];
	portCHAR jsontempv1[TEMP_JSON_LEN];
	memset(head,'\0',sizeof(head));
	memset(tail,'\0',sizeof(tail));
	static uint8_t count = 0;
	err_t err;
	while (1) {
		Err = xQueueReceive(xQueueUplinkHandle, &xQueueMbMqtt,portDEFAULT_WAIT_TIME*3);
		if (Err == pdPASS) {
			if (xQueueMbMqtt.gotflagProvision == 1 && xQueueMbMqtt.sum_dev != 0)  // check provision ?
				{
					err_t err;
					uint8_t SUM = xQueueMbMqtt.sum_dev;
				    for (uint8_t i = 0; i < SUM; i++)// duyệt từng phần tử trong mảng
				    {
				        for (uint8_t j = 0; j < SUM; j++)  // duyệt ID device
				        {
				            if ((dynamic+j)->deviceID == (dynamic+i)->deviceID && (dynamic+i)->deviceID != (dynamic+i-1)->deviceID){
				                    count ++;
				                    if (count == 1){
				                        if (i > 0){
				                        for (uint8_t a = 0; a < i; a++){
				                             if ((dynamic+a)->deviceID == (dynamic+j)->deviceID){
				                                goto TEST;
				                                }
				                            }
				                        }
				                        head_provision(head,(dynamic+j)->deviceID,(dynamic+j)->deviceName,(dynamic+j)->deviceType);
				                        for (uint8_t z = 0; z < SUM; z++)  // duyệt channel ID
				                        {
				                            if ((dynamic+z)->deviceID == (dynamic+j)->deviceID){
				                            	tail_provision(tail,(dynamic+z)->deviceChannel,(dynamic+z)->channeltitle,(dynamic+z)->valueType, (dynamic+z)->func);
				                                strcat(head,tail);
				                            }
				                        }
				                        head[strlen(head) - 1] = '\0';
				                        strcat(head,"]}]}");
				                        err = mqtt_publish(client,pro_topic, head,strlen(head), QOS_0, 0,mqtt_bridge_provision_request_cb,NULL);
				                        memset(head,'\0',sizeof(head));
										if (err != ERR_OK) {
											printf("\r\n Publish Provision err: %d\n", err);
											}
										osDelay(200);
				                    }
				            }
				        }
				        TEST: count = 0;
				    }
					BaseType_t er = pdFALSE;
					er = xQueueSend(xQueueDownlinkHandle, &xQueueMbMqtt,portDEFAULT_WAIT_TIME);
					if (er == pdPASS) {
					}
				}/* --------------END OF SENDING PROVISION---------------------------------------------------------------------*/
			else if (xQueueMbMqtt.gotflagcommand == 3){  // check command
				if (xQueueMbMqtt.FunC == 3)
					command_read_json(head, xQueueMbMqtt.NodeID, xQueueMbMqtt.RegAdr.i16data, xQueueMbMqtt.RegData.i16data);
				else if (xQueueMbMqtt.FunC == 6)
					command_write_json(head, xQueueMbMqtt.NodeID, xQueueMbMqtt.RegAdr.i16data);
				err = mqtt_publish(client,command_topic, head,strlen(head), QOS_0, 0,mqtt_bridge_command_request_cb,NULL);
				if (err != ERR_OK) {
					printf("\r\n Publish command err: %d\n", err);
					}
				else if (err == ERR_OK){
					xQueueMbMqtt.gotflagcommand = 0;
				}
			}/* --------------END OF SENDING COMAMND RESPONSE----------------------------------------------*/
			else if (xQueueMbMqtt.gotflagLast == 2){
				getTime(time);
				timestamp_telemetry(head,time);
				tail[strlen(tail) - 1] = '\0';
				strcat(tail,"}}}");
				strcat(head,tail);
				err = mqtt_publish(client, pub_topic,head,strlen(head), QOS_0, 0,mqtt_bridge_pub_request_cb,NULL);
				if (err != ERR_OK) {
					printf("\r\n Publish err: %d\n", err);
					if (err == -11){
						//MX_LWIP_Init();
					}
				}
				else if (err == ERR_OK){
					memset(head,'\0',sizeof(head));
					memset(tail,'\0',sizeof(tail));
					counter = 0;
				}
			}
			else if (xQueueMbMqtt.gotflagtelemetry == 2) { // check telemetry
			printf("\r\nTelemetry data: %d \t %d \t %d\r\n",xQueueMbMqtt.NodeID,xQueueMbMqtt.RegAdr.i16data ,xQueueMbMqtt.RegData.i16data);
			counter ++;
			if (counter == 1) {
				id_temp[0] = xQueueMbMqtt.NodeID;
				head_telemetry(jsontemp, xQueueMbMqtt.NodeID);
				strcat(tail,jsontemp);
				if (telemetry == 1){
					strcat(tail,jsontempv1);
					telemetry = 0;
				}
				tail_telemetry(jsontemp,xQueueMbMqtt.RegAdr.i16data, xQueueMbMqtt.RegData.i16data);
				strcat(tail,jsontemp);
			}
			if (counter > 1){
				id_temp[1] = xQueueMbMqtt.NodeID;
				if (id_temp[0] == id_temp[1]) {
					tail_telemetry(jsontemp,xQueueMbMqtt.RegAdr.i16data, xQueueMbMqtt.RegData.i16data);
					strcat(tail,jsontemp);
				}
				else if (id_temp[0] != id_temp[1]){
					telemetry = 1;
					counter = 0;
					tail_telemetry(jsontempv1,xQueueMbMqtt.RegAdr.i16data, xQueueMbMqtt.RegData.i16data);
					tail[strlen(tail) - 1] = '\0';
					strcat(tail,"},");
				}
			}
		}/* --------------END OF SENDING TELEMETRY--------------------------------------------------------------------*/
	} else {
			/*Create Json and publish to mqtt */
//			memset(publish_buffer, 0, MAX_JSON_LEN);
//			strcat(publish_buffer, "{\"NodeID\":");
//			strcat(publish_buffer, itoa_user(0, 10));
//			strcat(publish_, ",\"FunC\":");
//			strcat(publish_buffer, itoa_user(MB_FUNC_ACK, 10));
//			strcat(publish_buffer, ",\"RegAdrH\":");
//			strcat(publish_buffer,
//					itoa_user(xQueueMbMQtt.RegAdr.i8data[1], 10));
//			strcat(publish_buffer, ",\"RegAdrL\":");
//			strcat(publish_buffer,
//					itoa_user(xQueueMbMQtt.RegAdr.i8data[0], 10));
//			strcat(publish_buffer, ",\"RegDataH\":");
//			strcat(publish_buffer,
//					itoa_user(xQueueMbMQtt.RegData.i8data[1], 10));
//			strcat(publish_buffer, ",\"RegDataL\":");
//			strcat(publish_buffer,
//					itoa_user(xQueueMbMQtt.RegData.i8data[0], 10));
//			strcat(publish_buffer, ",\"PortID\":");
//			strcat(publish_buffer, itoa_user(xQueueMbMQtt.PortID, 10));
//			strcat(publish_buffer, "}");
////			HAL_UART_Transmit(&huart1, publish_buffer, MAX_JSON_LEN, 1000);
//			err = mqtt_publish(client, pub_topic, publish_buffer,
//					strlen(publish_buffer), QOS_0, 0,
//					mqtt_bridge_pub_request_cb,
//					NULL);
//			if (err != ERR_OK) {
//				mqtt_couter_err++;
//
//				printf("\r\n Publish err: %d\n", err);
//				if (mqtt_couter_err == 10) NVIC_SystemReset();
//			}
//			else
//			{
//				mqtt_couter_err = 0;
//			}

		}
//		vTaskDelay(10);

	}
}
//	how to got mqtt intance??
/*-------------------------------------PROVISION DOWNSTREAM AND COMMAND DOWNSTREAM---------------------------------------------------------------------------------------*/
uint8_t mqtt_modbus_thread_down_provision(char *Buffer,uint16_t BufferLen) {

		//printf("\r\n NUMBER OF DEVICE: %d\r\n",num_device);
		uint8_t  deviceID ,channelStatus;
		uint16_t channel_id;
		int r;
		jsmn_parser p;
		jsmntok_t t[JSON_MAX_LEN]; /* We exect no morep than JSON_MAX_LEN tokens */
		jsmn_init(&p);
		r = jsmn_parse(&p, Buffer, BufferLen, t,sizeof(t) / sizeof(t[0]));
		//printf("\r\n r value: %d\r\n", r);
		if (r < 0) {
			printf("Failed to parse JSON: %d\n", r);
			return 1;
		}
		if (r < 1 || t[0].type != JSMN_OBJECT) {
			printf("Object expected\n");
			return 1;
		}
		for (uint8_t i = 0; i < r-1; i++) {
			printf("\r\n -value: %.*s\r\n", t[i + 1].end - t[i + 1].start,Buffer + t[i + 1].start);
//			if (i == 2){
//				deviceID = atoi(Buffer + t[i + 1].start);
//			}else if (i == 5){
//				//printf("\r\n - channelID: %.*s\r\n", t[i + 1].end - t[i + 1].start,Buffer + t[i + 1].start);
//			}
//			else if ( i == 6){
//				for (uint8_t j = i; j < r-1; j++){
//					if(j % 2 == 0){
//						channel_id = atoi(Buffer + t[j + 1].start);
//					}
//					else if (j %2 != 0){
//						channelStatus = atoi(Buffer + t[j + 1].start);
//						for (uint8_t m = 0; m < num_device; m++){
//							if( (dynamic+m)->deviceID == deviceID && (dynamic+m)->deviceChannel == channel_id){
//								//(dynamic+m)->devicestatus = channelStatus;
//								printf("\r\n - deviceID %d \t channelID: %d \t channelstatus: %d\r\n",deviceID,channel_id,channelStatus);
//							}
//						}
//					}
//				}
//			}

		}
}
uint8_t mqtt_modbus_thread_down_command(char *pJsonMQTTBuffer,uint16_t pJsonMQTTBufferLen) {

		int r;
		jsmn_parser p;
		jsmntok_t t[JSON_MAX_LEN]; /* We expect no more than JSON_MAX_LEN tokens */
		jsmn_init(&p);
		xQueueMbMqtt_t xQueueMbMqtt;
		r = jsmn_parse(&p, pJsonMQTTBuffer, pJsonMQTTBufferLen, t,sizeof(t) / sizeof(t[0]));
		if (r < 0) {
			printf("Failed to parse JSON: %d\n", r);
			return 1;
		}

		/* Assume the top-level element is an object */
		if (r < 1 || t[0].type != JSMN_OBJECT) {
			printf("Object expected\n");
			return 1;
		}
		for (uint8_t i = 1; i < r; i++) {
			if (jsoneq(pJsonMQTTBuffer, &t[i], "device_id") == 0) {
				printf("\r\n - device_id: %.*s\n", t[i + 1].end - t[i + 1].start,pJsonMQTTBuffer + t[i + 1].start);
				xQueueMbMqtt.NodeID = atoi(pJsonMQTTBuffer + t[i + 1].start);
				i++;
			}
			else if (jsoneq(pJsonMQTTBuffer, &t[i], "channel_id") == 0) {
				printf("\r\n - channel_id: %.*s\n", t[i + 1].end - t[i + 1].start,pJsonMQTTBuffer + t[i + 1].start);
				xQueueMbMqtt.RegAdr.i16data = atoi(pJsonMQTTBuffer + t[i + 1].start);
				i++;
			}
			else if (jsoneq(pJsonMQTTBuffer, &t[i], "command") == 0) {
				uint8_t command = 0;
				char cmd[20];
				printf("\r\n - command: %.*s\n", t[i + 1].end - t[i + 1].start,pJsonMQTTBuffer + t[i + 1].start);
				strncpy(cmd, pJsonMQTTBuffer + t[i + 1].start, t[i + 1].end - t[i + 1].start);
				if (strstr(cmd,"read") != NULL){
					command = 3;
					uint16_t value = 1;
					xQueueMbMqtt.RegData.i8data[0] = (uint8_t)value;
					xQueueMbMqtt.RegData.i8data[1] = (uint8_t)(value >> 8);
				}
				else if (strstr(cmd,"write") != NULL)
					command = 6;
				xQueueMbMqtt.FunC = command;
				i++;
			}
			else if (jsoneq(pJsonMQTTBuffer, &t[i], "value") == 0) {
				uint16_t value = 0;
				char val[10];
				printf("\r\n - value: %.*s\n", t[i + 1].end - t[i + 1].start,pJsonMQTTBuffer + t[i + 1].start);
				strncpy(val, pJsonMQTTBuffer + t[i + 1].start, t[i + 1].end - t[i + 1].start);
				if (strstr(val,"true") != NULL || strstr(val,"on") != NULL)
					value = 1;
				else if (strstr(val,"false") != NULL || strstr(val,"off") != NULL)
					value = 0;
				else
					value = atoi(pJsonMQTTBuffer + t[i + 1].start);
				xQueueMbMqtt.RegData.i8data[0] = (uint8_t)value;
				xQueueMbMqtt.RegData.i8data[1] = (uint8_t)(value >> 8);
				i++;
			}
		}
		BaseType_t Err = pdFALSE;
		Err = xQueueSend(xQueueDownlinkHandle, &xQueueMbMqtt,portDEFAULT_WAIT_TIME);
		if (Err == pdPASS) {
			memset(pJsonMQTTBuffer,'\0',pJsonMQTTBufferLen);
		} else {
			printf("\r\n Modbus_MQTT Downlink queued: False \r\n");
		}
}

