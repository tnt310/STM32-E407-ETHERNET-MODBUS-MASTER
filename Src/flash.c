/* Built-in C library includes ----------------------*/

#include <flash.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
/* Platform includes --------------------------------*/
#include "main.h"
#include "mqttclienttask.h"
/* Private typedef -----------------------------------*/

typedef uint8_t xMemHandler_t;

/* Private define -----------------------------------*/
#define MEM_MAX_SIZE	(40)
#define MEM_RST_VAL
#define MEM_TEMP_OFF    4
//static xMemHandler_t xMemSharedHandler[MEM_MAX_SIZE];
static uint32_t uiAdr = FLASH_PAGE_ADDRESS;



/* Shared Variable ----------------------------------*/

host_param_t hostParam;
port_param_t portParam[MB_MAX_PORT];
network_param_t netParam;
network_param_t mqttHostParam;
uint32_t 		netId;
uint32_t 		modbus_mutex;
uint32_t 		modbus_telemetry;
uint32_t 		mqtt_port;
uint32_t 		timeDelay; // timeout: 15s, 30s, 1p, 3p, 5p, 10p
/* Start Implementation -----------------------------*/

/**
 * Brief: Get data
 * uiFuncAdr:
 * retVal: Value of Mem at uiFuncAdr
 *
 */
void uiFlashGet(uint32_t uiFlashAdr, uint32_t *ptrValue) {
	*ptrValue = *((__IO uint32_t*) uiFlashAdr);

}
/**
 * Brief: Set data
 * uiFuncAdr:
 * retVal: None
 *
 */
void uiFlashSet(uint32_t uiFlashAdr, uint32_t *ptrValue) {

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, uiFlashAdr, *ptrValue);

}
/**
 * Brief: Load stored data in flash
 *
 * retVal: None
 *
 */
void xFlashLoad(void) {

	/*Check if data have already saved*/
	uiAdr = FLASH_EXIST_ADDRESS;
	uint32_t flashExist = 0;
	uiFlashGet(uiAdr, (uint32_t*) &flashExist);
	if (flashExist == 1) {
		/*Network*/
//		uiAdr = FLASH_NET_ADDRESS  - MEM_TEMP_OFF;
//		uiFlashGet(uiAdr += 4, (uint32_t*) &netParam.ip);
//		uiFlashGet(uiAdr += 4, (uint32_t*) &netParam.netmask);
//		uiFlashGet(uiAdr += 4, (uint32_t*) &netParam.gateway);
//
//		/*Network MQTT Server*/
//		uiAdr = FLASH_HOST_ADDRESS  - MEM_TEMP_OFF;
//		uiFlashGet(uiAdr += 4, (uint32_t*) &mqttHostParam.ip);
//		uiFlashGet(uiAdr += 4, (uint32_t*) &mqttHostParam.netmask);
//		uiFlashGet(uiAdr += 4, (uint32_t*) &mqttHostParam.gateway);
//
//		uiAdr = FLASH_MUTEX_ADDRESS  - MEM_TEMP_OFF; // FOR MODBUS TELEMETRY
//		uiFlashGet(uiAdr += 4, (uint32_t*) &modbus_telemetry);
//
//		uiAdr = FLASH_TIMEOUT_ADDRESS  - MEM_TEMP_OFF; //FOR TIMEOUT OF MODBUS TASK
//		uiFlashGet(uiAdr += 4, (uint32_t*) &timeDelay);
//
//		uiAdr = FLASH_MQTTPORT_ADDRESS - MEM_TEMP_OFF; //FOR MQTT PORT
//		uiFlashGet(uiAdr += 4, (uint32_t*) &u16_mqtt_port);

		printf("\r\n Loaded pre-config from Flash \r\n");
	} else {
		printf("\r\n Flash have no data! Load default-config \r\n");
		netParam.ip.cdata[0] = 192;
		netParam.ip.cdata[1] = 168;
		netParam.ip.cdata[2] = 1;
		netParam.ip.cdata[3] = 20;
		netParam.netmask.cdata[0] = 255;
		netParam.netmask.cdata[1] = 255;
		netParam.netmask.cdata[2] = 255;
		netParam.netmask.cdata[3] = 0;
		netParam.gateway.cdata[0] = 192;
		netParam.gateway.cdata[1] = 168;
		netParam.gateway.cdata[2] = 1;
		netParam.gateway.cdata[3] = 1;

		mqttHostParam.ip.cdata[0] = 192;
		mqttHostParam.ip.cdata[1] = 168;
		mqttHostParam.ip.cdata[2] = 1;
		mqttHostParam.ip.cdata[3] = 24;
		mqttHostParam.netmask.cdata[0] = 255;
		mqttHostParam.netmask.cdata[1] = 255;
		mqttHostParam.netmask.cdata[2] = 255;
		mqttHostParam.netmask.cdata[3] = 0;
		mqttHostParam.gateway.cdata[0] = 192;
		mqttHostParam.gateway.cdata[1] = 168;
		mqttHostParam.gateway.cdata[2] = 1;
		mqttHostParam.gateway.cdata[3] = 1;
		netId = 01;
		xFlashSave();

	}

}
/**
 * Brief: Save data to flash
 *
 * retVal: None
 *
 */
void xFlashSave(void) {

	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.Sector = FLASH_SECTOR_7;
	EraseInitStruct.NbSectors = 1;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	uint32_t PageError = 0;
	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

//	/*Network*/
//	uiAdr = FLASH_NET_ADDRESS  - MEM_TEMP_OFF;
//	uiFlashSet(uiAdr += 4, (uint32_t*) &netParam.ip);
//	uiFlashSet(uiAdr += 4, (uint32_t*) &netParam.netmask);
//	uiFlashSet(uiAdr += 4, (uint32_t*) &netParam.gateway);
////
////	/*MQTT Network*/
//	uiAdr = FLASH_HOST_ADDRESS  - MEM_TEMP_OFF;
//	uiFlashSet(uiAdr += 4, (uint32_t*) &mqttHostParam.ip);
//	uiFlashSet(uiAdr += 4, (uint32_t*) &mqttHostParam.netmask);
//	uiFlashSet(uiAdr += 4, (uint32_t*) &mqttHostParam.gateway);
//
//
//	uiAdr = FLASH_MUTEX_ADDRESS  - MEM_TEMP_OFF;  // Modbus Mutex
//	uiFlashSet(uiAdr += 4, (uint32_t*) &modbus_mutex);
//
//	uiAdr = FLASH_TIMEOUT_ADDRESS  - MEM_TEMP_OFF;  // Timeout for Mdtask
//	uiFlashSet(uiAdr += 4, (uint32_t*) &timeDelay);
//
//	uiAdr = FLASH_MQTTPORT_ADDRESS  - MEM_TEMP_OFF;  // Mqtt_port
//	uiFlashSet(uiAdr += 4, (uint32_t*) &u16_mqtt_port);



	uiAdr = FLASH_EXIST_ADDRESS;
	uint32_t uiFashExist = 1;
	uiFlashSet(uiAdr, &uiFashExist);
	HAL_FLASH_Lock();
	printf("\r\n Flash saved successful  \r\n");
}

/*Function*/
/**
 * @brief
 * @param
 * @retval None
 */
char *itoa_user(uint64_t val, uint8_t base) {
	static uint8_t buf[32] = { 0 };  // 32 bits
	int i = 30;
	if (val == 0)
		buf[i--] = '0';
	for (; val && i; --i, val /= base)
		buf[i] = "0123456789abcdef"[val % base];

	return &buf[i + 1];
}

