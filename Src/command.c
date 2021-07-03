#include <command.h>
#include "cmdline.h"
#include "usart.h"
#include "flash.h"
#include "FreeRTOS.h"
#include "main.h"
#include "param.h"
#include "lwip.h"
#include "socket.h"
#include "lwip/ip4_addr.h"
#include "modbus_mqtt_bridge.h"
#include "fatfs.h"
#include <stdlib.h>
#include <stdio.h>
#include "sdcard.h"
#include "time_value.h"

extern network_param_t netParam;
extern network_param_t mqttHostParam;
extern osThreadId netFlashSave;
extern osThreadId defaultTaskHandle;
extern osSemaphoreId resetHandle;
extern osMessageQId xQueueResetHandle;
extern osMessageQId xQueueUplinkHandle;
extern data1_t *dynamic;

UART_HandleTypeDef huart6;
uint8_t num_device;
FATFS fs;
FIL fil;
FIL fil_temp;
FRESULT fresult;
DIR dirOject;
FILINFO fileInfo;
char SDbuffer[200];

uint32_t mqtt_port;
uint32_t modbus_mutex;
uint32_t timeDelay;
uint32_t port0_baud,port0_stop,port0_databit,port0_parity;
uint32_t port1_baud,port1_stop,port1_databit,port1_parity;

char *config_file = "config.txt";
char *device_file = "device.txt";
char *mqtt_file = "mqtt.txt";
char *serial_file = "serial.txt";

int Cmd_set_timeout(int argc, char *argv[]);
int Cmd_set_port0(int argc, char *argv[]);
int Cmd_set_port1(int argc, char *argv[]);
int Cmd_set_network(int argc, char *argv[]);
int Cmd_set_mqttInfo(int argc, char *argv[]);
int Cmd_get_time(int argc, char *argv[]);
int Cmd_off_mutex(int argc, char *argv[]);
int Cmd_set_mutex(int argc, char *argv[]);
int Cmd_set_localip(int argc, char *argv[]);
int Cmd_set_mqttip(int argc, char *argv[]);
int Cmd_mqtt_port(int argc, char *argv[]);
int Cmd_save(int argc, char *argv[]);
int Cmd_set_localgw(int argc, char *argv[]);
int Cmd_set_netmask(int argc, char *argv[]);
int Cmd_send_provision(int argc, char *argv[]);
int Cmd_allocate_device(int argc, char *argv[]);
int Cmd_delete_device(int argc, char *argv[]);
int Cmd_set_device(int argc, char *argv[]);
int Cmd_set_config(int argc, char *argv[]);
int Cmd_set_factory(int argc, char *argv[]);
int Cmd_delete_file(int argc, char *argv[]);
int Cmd_new_file(int argc, char *argv[]);
int Cmd_read_file(int argc, char *argv[]);
int Cmd_list_file(int argc, char *argv[]);
int Cmd_set_channelstatus(int argc, char *argv[]);
int Cmd_delete_channel(int argc, char *argv[]);
int Cmd_delete_line(int argc, char *argv[]);
int Cmd_check_record(int argc, char *argv[]);

tCmdLineEntry g_psCmdTable[] = {
		{ "checkrecord",Cmd_check_record," : Send provision request" },
		{ "deleteline",Cmd_delete_line," : Send provision request" },
		{ "setfactory",Cmd_set_factory," : Send provision request" },
		{ "configtimeout",Cmd_set_timeout," : Send provision request" },
		{ "configport0",Cmd_set_port0," : Send provision request" },
		{ "configport1",Cmd_set_port1," : Send provision request" },
		{ "confignetwork",Cmd_set_network," : Send provision request" },
		{ "configmqtt",Cmd_set_mqttInfo," : Send provision request" },
		{ "gettime",Cmd_get_time," : Send provision request" },
		{ "setmqttport", Cmd_mqtt_port," : Set static ip for brigde" },
		{ "setip", Cmd_set_localip," : Set static ip for brigde" },
		{ "setmqttip", Cmd_set_mqttip," : Set mqtt server ip for brigde" },
		{ "save", Cmd_save," : Save all configuration to flash and reboot" },
		{ "setgateway", Cmd_set_localgw," : Set default gateway for board" },
		{ "setnetmask", Cmd_set_netmask," : Set netmask for board" },
		{ "sendprovision", Cmd_send_provision," : Send provision request" },
		{ "loadtable", Cmd_allocate_device," : Update network" },
		{ "setmutex", Cmd_set_mutex," : Update network" },
		{ "offmutex", Cmd_off_mutex," : Send provision request" },
		{ "check",Cmd_delete_device," : Update network" },
		{ "deletechannel",Cmd_delete_channel," : Send provision request" },
		{ "setdevice", Cmd_set_device," : Send provision request" },
		{ "setconfig", Cmd_set_config," : Send provision request" },
		{ "touch",Cmd_new_file," : Send provision request" },
		{ "rm",Cmd_delete_file," : Send provision request" },
		{ "nano",Cmd_read_file," : Send provision request" },
		{ "ls",Cmd_list_file," : Send provision request" },

		{ 0, 0, 0 } };

const char * ErrorCode[4] = { "CMDLINE_BAD_CMD", "CMDLINE_TOO_MANY_ARGS",
		"CMDLINE_TOO_FEW_ARGS", "CMDLINE_INVALID_ARG" };

uint8_t commandBuffer[200];
uint32_t commandBufferIndex = 0;
uint32_t gotCommandFlag = 0;

void UARTIntHandler(void) {
	uint8_t receivedChar;
	char *EnterCMD = "\r\n>";
	receivedChar = (uint8_t) ((huart6).Instance->DR & (uint8_t) 0x00FF);
	HAL_UART_Transmit(&huart6, &receivedChar, 1, 100);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
	if (receivedChar != 13) {
		if ((receivedChar == 8) || (receivedChar == 127)) {
			if (commandBufferIndex > 0)
				commandBufferIndex--;
		} else {
			commandBuffer[commandBufferIndex] = receivedChar;
			commandBufferIndex++;
		}
	} else {
		if (commandBufferIndex != 0) {
			commandBuffer[commandBufferIndex] = 0;
			commandBufferIndex = 0;
			gotCommandFlag = 1;
		}
		HAL_UART_Transmit(&huart6,(uint8_t*)EnterCMD, 3, 100);
	}

}
/*---------------------------FILE COMMAND-----------------------------------------------------------------------*/
int Cmd_new_file(int argc, char *argv[])
{
	printf("\r\nCmd_new_file\r\n");
	printf("------------------\r\n");
	uint8_t status = 0;
	char *file = *(argv+1);
	MX_FATFS_Init();
	if (f_mount(&fs, "/", 1) == FR_OK){
		if(f_open(&fil,file, FA_CREATE_ALWAYS) == FR_OK){
			if (f_close(&fil) == FR_OK){
				status = 1;
			}
		}else if (f_open(&fil,"config.txt", FA_CREATE_ALWAYS) != FR_OK){
			printf("\r\nCREATE FILE FAIL CMN\r\n");
		}
	}else if (f_mount(&fs, "/", 1) != FR_OK){
		printf("\r\nNOT MOUTING SD CARD, PLEASE CHECK SD CARD\r\n");
	}
	printf("\r\n create newfile status :%d\r\n",status);
}
int Cmd_delete_file(int argc, char *argv[])
{
	printf("\nCmd_delete_file\r\n");
	printf("------------------\r\n");
	uint8_t status = 0;
	char *file= *(argv+1);
	MX_FATFS_Init();
	if(f_opendir (&dirOject,"/") == FR_OK){
		if(f_unlink(file) == FR_OK){
			if(f_closedir(&dirOject) == FR_OK){
				status = 1;
			}
		}else if(f_unlink(file) != FR_OK){
			printf("\r\nDELETE FILE FAIL\r\n");
		}
	}else if(f_opendir(&dirOject,"/") != FR_OK){
		printf("\r\nNOT MOUTING DIRECTORY\r\n");
	}
	printf("\r\n delete file status: %d\r\n",status);
}
int Cmd_list_file(int argc, char *argv[])
{
	printf("\r\nCmd_list_file\r\n");
	printf("------------------\r\n");
	uint8_t file = 0;
	MX_FATFS_Init();
	if(f_opendir (&dirOject,"/") == FR_OK){
		if((f_readdir(&dirOject, &fileInfo) != FR_OK) || (fileInfo.fname[0] == 0)){
			printf("\r\nOPEN FILE FAIL\r\n");
		}else{
			for (uint8_t i = 0; (f_readdir(&dirOject, &fileInfo) == FR_OK) && (fileInfo.fname[0] != 0) ; i++){
					file++;
					printf("\r\nfile %d: %s",file,fileInfo.fname);
			}
				printf("\r\nNum_file: %d\r\n",file);
				if (f_closedir(&dirOject) == FR_OK){
					printf("\r\nList file status : 1\r\n");
				}
		}
	}else if(f_opendir(&dirOject,"/") != FR_OK){
		printf("\r\nNOT MOUTING DIRECTORY\r\n");
	}
}
int Cmd_read_file(int argc, char *argv[])
{
	printf("\r\nCmd_read_file\r\n");
	printf("------------------\r\n");
	uint8_t i, status = 0;
	char *file= *(argv+1);
	if (f_mount(&fs, "/", 1) == FR_OK){
		if(f_open(&fil,file, FA_READ) == FR_OK){
			for (i = 0; (f_eof(&fil) == 0); i++){
				memset(SDbuffer,'\0',sizeof(SDbuffer));
				f_gets((char*)SDbuffer, sizeof(SDbuffer), &fil);
				printf("\r\n File: %s Line %d: %s",file,i,SDbuffer);
			}
		}else if(f_open(&fil,file, FA_READ) != FR_OK){
			printf("\r\nNOT OPEN FILE\r\n");
		}
		if (f_close(&fil) == FR_OK){
			status = 1;
		}
	}else if (f_mount(&fs, "/", 1) != FR_OK){
		printf("\r\nNOT MOUTING SD CARD, PLEASE CHECK SD CARD\r\n");
	}
	printf("\r\n Read file status :%d\r\n",status);
}
int Cmd_delete_channel(int argc, char *argv[])
{
	printf("\r\nCmd_delete_channel\r\n");
	printf("------------------\r\n");
	uint8_t id = atoi(*(argv+1));
	for (uint8_t i = 0 ; i < num_device; i++){

	}

	}
/*---------------------------SET PARAMETER INTO SDCARD-----------------------------------------------------------------------*/
static uint8_t write_sdcard(char *file,char *buffer)
{
	uint8_t status = 0;
	MX_FATFS_Init();
	if (f_mount(&fs, "/", 1) == FR_OK){
		if(f_open(&fil,file, FA_OPEN_ALWAYS|FA_WRITE) == FR_OK){
			if (f_lseek(&fil, f_size(&fil)) == FR_OK){
				f_puts(buffer, &fil);
				if (f_close(&fil) == FR_OK){
					status = 1;
				}
			}
		}
	}else if (f_mount(&fs, "/", 1) != FR_OK){
		printf("\r\nNOT MOUTING SD CARD, PLEASE CHECK SD CARD\r\n");
	}
	return status;
}
/*
 * - group 1: mqtt parameter
 * - group 2: rs232 parameter
 * - group 3: modbus port parameter
 * */
int Cmd_set_config(int argc, char *argv[])
{
	uint8_t status = 0;
	printf("\r\nCmd_set_config\r\n");
	printf("------------------\r\n");
	char buffer[200];
	uint8_t group = atoi(*(argv+1));
	if (group == 1 ){/* {"mqttId":"null","username":"null","pwd":"null","port":1883,"apikey":"60cda6bc55193093bbcd001f"}*/
		memset(buffer,'\0',sizeof(buffer));
		char *mqttId  = *(argv+2);
		char *username = *(argv+3);
		char *pwd = *(argv+4);
		uint16_t mqtt_port = atoi(*(argv+5));
		char *apikey = *(argv+6);
		SD_Mqtt(buffer, mqtt_port, mqttId, username, pwd, apikey);
		if (write_sdcard("mqtt.txt", buffer) == 1){
			status = 1;
		}else if (write_sdcard("mqtt.txt", buffer) == 0){
			printf("\nWRITE DATA INTO SD CARD FAIL CMNR\r\n");
		}
	}else if(group == 2){/* {"rs232":{"baud":115200,"databits":8,"stopbits":1,"parity":0}} */
		memset(buffer,'\0',sizeof(buffer));
		uint8_t type_serial = atoi(*(argv+2));
		uint8_t baud = atoi(*(argv+3));
		uint8_t databits = atoi(*(argv+4));
		uint8_t stopbit = atoi(*(argv+5));
		uint8_t parity = atoi(*(argv+6));
		SD_Serial(buffer, type_serial, baud, databits, stopbit, parity);
		if (write_sdcard("serial.txt", buffer) == 1){
			status = 1;
		}else if (write_sdcard("serial.txt", buffer) == 0){
			printf("\nWRITE DATA INTO SD CARD FAIL CMNR\r\n");
		}
	}else if(group ==3 ){
		memset(buffer,'\0',sizeof(buffer));
		uint8_t type_serial = atoi(*(argv+2));
		uint8_t baud = atoi(*(argv+3));
		uint8_t databits = atoi(*(argv+4));
		uint8_t stopbit = atoi(*(argv+5));
		uint8_t parity = atoi(*(argv+6));
		SD_Serial(buffer,type_serial, baud, databits, stopbit, parity);
		if (write_sdcard("serial.txt", buffer) == 1){
			status = 1;
		}else if (write_sdcard("serial.txt", buffer) == 0){
			printf("\nWRITE DATA INTO SD CARD FAIL CMNR\r\n");
		}
	}
	printf("\r\n Write status :%d\r\n",status);
}
int Cmd_set_device(int argc, char *argv[])
{
	printf("\nCmd_write_sdcard\r\n");
	printf("------------------\r\n");
	char buffer[200];
	uint8_t port= atoi(*(argv+1));
	uint8_t deviceID = atoi(*(argv+2));
	uint8_t func = atoi(*(argv+3));
	char *deviceChannel= *(argv+4);
	char *deviceType= *(argv+5);
	char *deviceName = *(argv+6);
	char *deviceTitle= *(argv+7);
	char *valueType= *(argv+8);
	uint8_t deviceStatus = 0;
	uint16_t scale = atoi(*(argv+9));
	SD_Device(buffer,port,deviceID,func,deviceChannel,deviceType,deviceTitle,deviceName,valueType,deviceStatus,scale);
	uint8_t status = write_sdcard("test.txt", buffer);
	if (status == 1){
		printf("\r\n write data status: %d\r\n",status);
	}else
		printf("\r\n write data status: %d\r\n",status);
	printf("\r\n%s",buffer);
}
int Cmd_delete_device(int argc, char *argv[])  //xem fb
{
	printf("\nCmd_delete_device\r\n");
	printf("------------------\r\n");
	memset(SDbuffer,'\0',sizeof(SDbuffer));
	MX_FATFS_Init();
	if (f_mount(&fs, "", 1) == FR_OK){
		if(f_open(&fil,"sdio.txt", FA_READ) == FR_OK){
			f_gets(SDbuffer, sizeof(SDbuffer),&fil);
			f_close(&fil);
			printf("\r\n SDbuffer: %s\r\n",SDbuffer);
		}
	}else if (f_mount(&fs, "", 1) != FR_OK){
		printf("\r\nNOT MOUTING SD CARD, PLEASE CHECK SD CARD\r\n");
	}
}
/*---------------------------SEND PROVISION-----------------------------------------------------------------------*/
int Cmd_send_provision(int argc, char *argv[])
{
	printf("\nCmd_send_provision\r\n");
	printf("------------------\r\n");
	printf("\r\n NUMBER OF DEVICE: %d\r\n",num_device);
	xQueueMbMqtt_t xQueueMbMqtt;
	BaseType_t Err = pdFALSE;
	#define portDEFAULT_WAIT_TIME 1000
	xQueueMbMqtt.gotflagProvision = 1;
	xQueueMbMqtt.sum_dev = num_device;
	Err = xQueueSend(xQueueUplinkHandle, &xQueueMbMqtt,portDEFAULT_WAIT_TIME);
		if (Err == pdPASS) {
			xQueueMbMqtt.gotflagProvision = 0;
			}
		else {
		printf("\r\n Modbus_MQTT Up queued: False \r\n");
		}
}
/*---------------------IP, NETMASK, GATEWAY, MQTT ID, MQTT PORT-------------------------------------------------------------------------*/
int Cmd_set_localgw(int argc, char *argv[]) {
	printf("\nCmd_set_gateway\r\n");
	printf("------------------\r\n");
	ip4_addr_t gateway;
	if (ipaddr_aton(*(argv + 1), &gateway)) {
		netParam.gateway.idata = gateway.addr;
		printf("\r\n New gateway: %d %d %d %d", netParam.gateway.cdata[0],
				netParam.gateway.cdata[1],netParam.gateway.cdata[2],
				netParam.gateway.cdata[3]);
	} else {
		printf("\r\nInvalid param\r\n");
	}
	printf("\r\n------------------\r\n");
	return 0;
}
int Cmd_set_netmask(int argc, char *argv[]) {
	printf("\nCmd_set_netmask\r\n");
	printf("------------------\r\n");
	ip4_addr_t netmask;
	if (ipaddr_aton(*(argv + 1), &netmask)) {
		netParam.netmask.idata = netmask.addr;
		printf("\r\n New netmask: %d %d %d %d", netParam.netmask.cdata[0],
				netParam.netmask.cdata[1],netParam.netmask.cdata[2],
				netParam.netmask.cdata[3]);
	} else {
		printf("\r\nInvalid param\r\n");
	}
	printf("\r\n------------------\r\n");
	return 0;
}
int Cmd_set_localip(int argc, char *argv[]) {
	printf("\nCmd_set_localip\r\n");
	printf("------------------\r\n");
	ip4_addr_t ip;
	if (ipaddr_aton(*(argv + 1), &ip)) {
		netParam.ip.idata = ip.addr;
		printf("\r\n New IP: %d %d %d %d", netParam.ip.cdata[0],
				netParam.ip.cdata[1], netParam.ip.cdata[2],
				netParam.ip.cdata[3]);
	} else {
		printf("\r\nInvalid param\r\n");
	}
	printf("\r\n------------------\r\n");
	return 0;
}
int Cmd_set_mqttip(int argc, char *argv[]) {
	printf("\nCmd_set_mqttip\r\n");
	printf("------------------\r\n");
	ip4_addr_t mqtt;
	if (ipaddr_aton(*(argv + 1), &mqtt)) {
		mqttHostParam.ip.idata = mqtt.addr;
		printf("\r\n New Broker IP: %d %d %d %d", mqttHostParam.ip.cdata[0],
				mqttHostParam.ip.cdata[1], mqttHostParam.ip.cdata[2],
				mqttHostParam.ip.cdata[3]);
	} else {
		printf("\r\nInvalid param\r\n");
		printf("\r\n------------------\r\n");
		return 0;
	}
}
int Cmd_mqtt_port(int argc, char *argv[])
{
	printf("\nCmd_mqtt_port\r\n");
	printf("------------------\r\n");
	mqtt_port= atoi(*(argv+1));
	printf("\r\n New mqtt port: %d",mqtt_port);
	printf("\r\n------------------\r\n");
	}
/*---------------------LOAD TABLE ALLOCATE MEMORY--------------------------------------------------------------------------*/
int Cmd_allocate_device(int argc, char *argv[])
{
	printf("\nCmd_load_table\r\n");
	printf("------------------\r\n");
	printf("\r\n NUMBER OF DEVICE: %d\r\n",num_device);
	printf("\r\nLOADING NEW DEVICE--------------------------------------------------------\r\n");
	for (uint8_t i = 0; i < num_device; i++)
	{
	   printf("\r\nDevice %d: %d\t%d\t%d\t%d\t%d\t%d\t%d\t%s\t%s\t%s\t%s",i,(dynamic+i)->channel,(dynamic+i)->deviceID,(dynamic+i)->func,(dynamic+i)->devicestatus,
			   (dynamic+i)->numreg,(dynamic+i)->scale,(dynamic+i)->deviceChannel,(dynamic+i)->deviceType,(dynamic+i)->deviceName,
			   (dynamic+i)->channeltitle,(dynamic+i)->valueType);
	}
	printf("\r\nADDRESS OFF NEW DEVICE--------------------------------------------------------\r\n");
	for (uint8_t i = 0; i < num_device; i++)
	{
		   printf("\r\nDevice %d: %d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d",i,&(dynamic+i)->channel,&(dynamic+i)->deviceID,&(dynamic+i)->func,&(dynamic+i)->devicestatus,
				   &(dynamic+i)->numreg,&(dynamic+i)->scale,&(dynamic+i)->deviceChannel,&(dynamic+i)->deviceType,&(dynamic+i)->deviceName,
				   &(dynamic+i)->channeltitle,&(dynamic+i)->valueType);
	}
	printf("\r\n");
}
/*---------------------SET MUTEX FOR MBTASK--------------------------------------------------------------------------*/
int Cmd_set_mutex(int argc, char *argv[])
{
	printf("\nCmd_set_telemetry\r\n");
	printf("------------------\r\n");
	modbus_mutex = 1;
	uint32_t handle = 1;
	xQueueSend(xQueueResetHandle,&handle,portMAX_DELAY);
	}
/*----------------------OFF MUTEX FOR MBTASK-------------------------------------------------------------------------*/
int Cmd_off_mutex(int argc, char *argv[]) {
	printf("\nCmd_off_telemetry\r\n");
	printf("------------------\r\n");
	modbus_mutex = 0;
	uint32_t handle = 1;
	xQueueSend(xQueueResetHandle,&handle,portMAX_DELAY);
}

uint8_t unlink(char *file)
{
	uint8_t status = 0;
	if(f_opendir (&dirOject,"/") == FR_OK){
		if(f_unlink(file) == FR_OK){
			if(f_closedir(&dirOject) == FR_OK){
				status = 1;
			}
		}else if(f_unlink(file) != FR_OK){
			printf("\r\nDELETE FILE FAIL\r\n");
		}
	}else if(f_opendir(&dirOject,"/") != FR_OK){
		printf("\r\nNOT MOUTING DIRECTORY\r\n");
	}
	printf("\r\n delete file status: %d\r\n",status);
	return status;
}
int Cmd_delete_line(int argc, char *argv[])
{
	printf("\nCmd_delete_line\r\n");
	printf("------------------\r\n");
	uint8_t lct = 0;
	char *file =*(argv+1);
	uint8_t line = (uint8_t)atoi(*(argv+2));
	char *data = "{Hello m con di ngu ngoc}\n";
	printf("\r\nFile: %s - Line :%d\r\n",file,line);
	MX_FATFS_Init();
	if (f_mount(&fs, "/", 1) == FR_OK){
		if(f_open(&fil,file, FA_OPEN_ALWAYS|FA_READ|FA_WRITE) == FR_OK){
			for (uint8_t i = 0; (f_eof(&fil) == 0); i++)
				{
					memset(SDbuffer,'\0',sizeof(SDbuffer));
					f_gets((char*)SDbuffer, sizeof(SDbuffer), &fil);
					if (lct == line){
						f_open(&fil_temp,"temp.txt", FA_OPEN_ALWAYS|FA_WRITE);
						f_lseek(&fil_temp, f_size(&fil_temp));
						f_puts(data, &fil_temp);
						f_close(&fil_temp);
						lct++;
					}
					else if (lct != line){
						lct++;
						f_open(&fil_temp,"temp.txt", FA_OPEN_ALWAYS|FA_WRITE);
						f_lseek(&fil_temp, f_size(&fil_temp));
						f_puts(SDbuffer, &fil_temp);
						f_close(&fil_temp);
					}
				}
			f_unlink(file);
			f_rename("temp.txt",file);
			f_close(&fil);
		}
	}else if (f_mount(&fs,"/", 1) != FR_OK) {
		printf("\r\nNOT MOUTING SD CARD, PLEASE CHECK SD CARD\r\n");
	}
}
int Cmd_set_factory(int argc, char *argv[])
{
	printf("\nCmd_set_factory\r\n");
	printf("------------------\r\n");
	port0_baud = 8;
	port0_databit = 0;
	port0_stop = 0;
	port0_parity = 0;
	USER_USART3_UART_Init();
	port1_baud = 8;
	port1_databit = 0;
	port1_stop = 0;
	port1_parity = 0;
	USER_USART2_UART_Init();
	printf("\r\n PORT 0: baud: %d, databits: %d, stopbits: %d, parity: %d\r\n",port0_baud,port0_databit, port0_stop, port0_parity);
	printf("\r\n PORT 1: baud: %d, databits: %d, stopbits: %d, parity: %d\r\n",port1_baud,port1_databit, port1_stop, port1_parity);
}
int Cmd_get_time(int argc, char *argv[])
{
	printf("\nCmd_get_time\r\n");
	printf("------------------\r\n");
	uint8_t time[6];
	getTime(time);
	printf("\r\nTime from RTC: %d\t %d\t %d\t %d\t %d\t %d\r\n",time[0],time[1],time[2],time[3],time[4],time[5]);
}
static uint8_t overwrite_file(char *file, char *data, uint8_t line)
{
	uint8_t lct = 0;
	MX_FATFS_Init();
	if (f_mount(&fs, "/", 1) == FR_OK){
		if(f_open(&fil,file,FA_OPEN_ALWAYS|FA_READ|FA_WRITE) == FR_OK){
			for (uint8_t i = 0; (f_eof(&fil) == 0); i++)
				{
					memset(SDbuffer,'\0',sizeof(SDbuffer));
					f_gets((char*)SDbuffer, sizeof(SDbuffer), &fil);
					if (lct == line){
						f_open(&fil_temp,"temp.txt", FA_OPEN_ALWAYS|FA_WRITE);
						f_lseek(&fil_temp, f_size(&fil_temp));
						f_puts(data, &fil_temp);
						f_close(&fil_temp);
						lct++;
					}
					else if (lct != line){
						lct++;
						f_open(&fil_temp,"temp.txt", FA_OPEN_ALWAYS|FA_WRITE);
						f_lseek(&fil_temp, f_size(&fil_temp));
						f_puts(SDbuffer, &fil_temp);
						f_close(&fil_temp);
					}
				}
			f_unlink(file);
			f_rename("temp.txt",file);
			f_close(&fil);
		}
	}else if (f_mount(&fs,"/", 1) != FR_OK) {
		printf("\r\nNOT MOUTING SD CARD, PLEASE CHECK SD CARD\r\n");
	}
}
int Cmd_set_timeout(int argc, char *argv[]) // timeout: 15s, 30s, 1p, 3p, 5p, 10p
{
	uint16_t timeout= atoi(*(argv+1));
	char buffer[20];
	SD_timeout(buffer,timeout);
	printf("\r\n timeout is set: %s\r\n",buffer);
	overwrite_file("test.txt",buffer, 4);
}
int Cmd_set_port0(int argc, char *argv[]) // timeout: 15s, 30s, 1p, 3p, 5p, 10p
{
	uint8_t baud= atoi(*(argv+1));
	uint8_t dbbits= atoi(*(argv+2));
	uint8_t stops= atoi(*(argv+3));
	uint8_t parity= atoi(*(argv+4));
	char buffer[100];
	SD_Serial(buffer,2, baud,dbbits, stops, parity);
	printf("\r\n port0 is set: %s\r\n",buffer);
	overwrite_file("test.txt",buffer, 2);
}
int Cmd_set_port1(int argc, char *argv[]) // timeout: 15s, 30s, 1p, 3p, 5p, 10p
{
	uint8_t baud= atoi(*(argv+1));
	uint8_t dbbits= atoi(*(argv+2));
	uint8_t stops= atoi(*(argv+3));
	uint8_t parity= atoi(*(argv+4));
	char buffer[100];
	SD_Serial(buffer,3, baud,dbbits, stops, parity);
	printf("\r\n port1 is set: %s\r\n",buffer);
	overwrite_file("test.txt",buffer, 3);
}
int Cmd_set_network(int argc, char *argv[]) // timeout: 15s, 30s, 1p, 3p, 5p, 10p
{
	char *ip = *(argv+1);
	char *netmask = *(argv+2);
	char *gateway = *(argv+3);
	char *broker = *(argv+4);
	char buffer[200];
	SD_Network(buffer, ip, netmask, gateway, broker);
	printf("\r\n network is set: %s\r\n",buffer);
	overwrite_file("test.txt",buffer, 0);
}
int Cmd_set_mqttInfo(int argc, char *argv[]) // timeout: 15s, 30s, 1p, 3p, 5p, 10p
{
	char *id = *(argv+1);
	char *username = *(argv+2);
	char *pwd = *(argv+3);
	uint16_t port = atoi(*(argv+4));
	char *apikey = *(argv+5);
	char buffer[200];
	SD_Mqtt(buffer, port,id, username, pwd, apikey);
	printf("\r\n mqttInfo is set: %s\r\n",buffer);
	overwrite_file("test.txt",buffer, 1);
}
/*---------------------------SAVE------------------------------------------------------------------------------*/
int Cmd_save(int argc, char *argv[]) {

	uint32_t handle = 1;
	xQueueSend(xQueueResetHandle,&handle,portMAX_DELAY);

}
/*---------------------------LOAD DATA FROM SDCARD------------------------------------------------------------*/
uint8_t LoadSdcard(char *file)
{
	uint8_t status = 0;
	uint8_t line = 0;
	MX_FATFS_Init();
	if (f_mount(&fs,"/", 1) == FR_OK){
		if (f_open(&fil,file, FA_READ) == FR_OK){
			for (line= 0; (f_eof(&fil) == 0); line++)
				{
					f_gets((char*)SDbuffer, sizeof(SDbuffer), &fil);
					parse_sdcardInfo(SDbuffer, strlen(SDbuffer));
					//printf("\r\n File %s Line %d: %s\r\n",file,line,SDbuffer);
				}
			if (f_close(&fil) == FR_OK){
				status = 1;
				printf("\r\n LOAD SDCARD STATUS : %d\r\n",status);
			}
		}else if (f_open(&fil,file, FA_READ) != FR_OK){
			printf("\r\n NOT OPEN FILE\r\n");
		}
	}else if (f_mount(&fs,"/", 1) != FR_OK){
		printf("\r\n NOT MOUTING SDCARD WHEN RUNIING\r\n");
	}
	return status;
}
/*---------------------------WRITE DATA TO RECORD------------------------------------------------------------*/
uint8_t RecordData(char *file, char *buffer)
{
	uint8_t status = 0;
	if (write_sdcard(file, buffer) == 1){
		status = 1;
	}else if (write_sdcard(file, buffer) == 0){
		printf("\nWRITE DATA INTO SD CARD FAIL CMNR\r\n");
	}
	return status;
}
uint8_t CheckRecord(char *file)
{
	uint8_t status = 0;
	uint8_t line = 0;
	MX_FATFS_Init();
	if (f_mount(&fs,"/", 1) == FR_OK){
		if (f_open(&fil,file, FA_READ) == FR_OK){
			if (f_eof(&fil) != 0){
				printf("\r\n NO DATA IN FILE RECORD.TXT\r\n");
			}else if(f_eof(&fil) == 0){
				status = 1;
				printf("\r\n HAVE DATA IN FILE RECORD.TXT\r\n");
			}
			f_close(&fil);
		}else if (f_open(&fil,file, FA_READ) != FR_OK){
			printf("\r\n NOT OPEN FILE\r\n");
		}
	}else if (f_mount(&fs,"/", 1) != FR_OK){
		printf("\r\n NOT MOUTING SDCARD WHEN RUNIING\r\n");
	}
	return status;
}
int Cmd_check_record(int argc, char *argv[])
{
	printf("\nCmd_get_time\r\n");
	printf("------------------\r\n");
	char *file =*(argv+1);
	CheckRecord(file);
	}
void ftoa(char buffer[20], char string[20], uint16_t scale)
{
	char temp, temp1, temp2;
	char val[20];
	memset(val,'\0',sizeof(val));
	strncpy(val,string,strlen(string));
	memset(buffer,'\0',sizeof(buffer));
	for (uint8_t i = 0; i < strlen(string); i++){
		buffer[i] = string[i];
	}
	if (scale == 10) {
		temp = buffer[strlen(string)-1];
		buffer[strlen(string)-1] = '.';
		buffer[strlen(string)] = temp;
	}else if (scale == 100) {
		temp = buffer[strlen(string) -2];
		temp1 = buffer[strlen(string) -1];
		buffer[strlen(string) -2] = '.';
		buffer[strlen(string) -1] = temp;
		buffer[strlen(string)] = temp1;
	}
	else if (scale == 1000){
		temp = buffer[strlen(string) -3]; // 3
		temp1 = buffer[strlen(string) -2]; // 4
		temp2 = buffer[strlen(string) -1];  // 5
		buffer[strlen(string) -3] = '.';
		buffer[strlen(string) -2] = temp;//3
		buffer[strlen(string) - 1] = temp1; //4
		buffer[strlen(string)] = temp2;  // 5
	}
}
