#ifndef _PERIPHERAL__H_
#define _PERIPHERAL__H_

/* Built-in C library includes ----------------------*/
#include <stdio.h>
#include <stdint.h>

/* Define */
#define PORT_MAX_DEV	1
#define MB_MAX_PORT		2
#define MB_DEFAUL_DEV	0


enum DEV_STATE {
	DEV_NOT_ACT,
	DEV_ACT,
	DEV_BUSY,
	DEV_ERR
};
enum PORT_STATE
{
	PORT_NOT_ACT,
	PORT_ACT
};
/* Private typedef prototype*/

typedef union
{
	uint32_t idata;
	uint8_t  cdata[4];
}int_to_byte;

typedef struct
{
	int_to_byte username[3];
	int_to_byte password[3];
}host_param_t;
typedef struct
{
	uint8_t uiAdr; /*Address indicate type of device*/
	uint8_t uiDevState; /**/

}port_device_t;
typedef struct
{
	port_device_t portDev[PORT_MAX_DEV];
	uint8_t uiPortState;
}port_param_t;

typedef struct
{
	int_to_byte ip;
	int_to_byte netmask;
	int_to_byte gateway;

}network_param_t;

extern uint32_t modbus_mutex;
extern uint32_t modbus_telemetry;
extern uint32_t mqtt_port;
extern uint32_t timeDelay;
uint32_t port0_baud,port0_stop,port0_databit,port0_parity;
uint32_t port1_baud,port1_stop,port1_databit,port1_parity;
/*Function Address define*/
#define FLASH_EXIST_ADDRESS   	0x08070000
#define FLASH_PAGE_ADDRESS    	0x08070008
#define FLASH_NET_ADDRESS	  	0x08070020
#define FLASH_HOST_ADDRESS    	0x08070040
#define FLASH_MUTEX_ADDRESS   	0x08070060
#define FLASH_TIMEOUT_ADDRESS	0x08070064
#define FLASH_MQTTPORT_ADDRESS	0x08070068

#define FLASH_PORT1_ADDRESS		0x08070072
#define FLASH_PORT2_ADDRESS		0x08070088

#define FLASH_SAVED_DATA      0x01


#define MEM_MAX_SIZE	(40)

/*Public Function prototype---------------------*/

void xFlashLoad(void) ;
void xFlashSave(void) ;

char *itoa_user(uint64_t val, uint8_t base);


#endif
