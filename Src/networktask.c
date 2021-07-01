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
extern host_param_t hostParam;
extern port_param_t portParam[MB_MAX_PORT];
extern network_param_t mqttHostParam;
extern UART_HandleTypeDef huart1;

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

void http_server_netconn_thread(void *arg);
void http_server_netconn_init(void);
void NetworkTimeTask(void const *arg);
void TcpDiscoverTask(void *arg);
void http_client(void *arg);

/* Private typedef -----------------------------------------------------------*/
ip4_addr_t server_addr;

/* Private define ------------------------------------------------------------*/
#define WEBSERVER_THREAD_PRIO (osPriorityNormal)

/* Shared Variables -----------------------------------------*/
osThreadId netTimeTask;
osThreadId netHTTPTask;
osThreadId netTcpEchoTask;
osThreadId netFlashSave;

UART_HandleTypeDef huart6;

extern osMessageQId xQueueControlHandle;
extern osMessageQId xQueueMessageHandle;
extern osSemaphoreId netMqttIpSemaphoreHandle;
extern osMessageQId xQueueDownlinkHandle;
extern osMessageQId xQueueUplinkHandle;
osMessageQId xQueueProvisionHandle;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u32_t nPageHits = 0;

char tempBuffer[100];
uint8_t uiUserPass[30];

uint8_t IS_LOGIN = 0;

extern RTC_HandleTypeDef hrtc;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief serve tcp connection
 * @param conn: pointer on connection structure
 * @retval None
 */
static void http_server_serve(struct netconn *conn) {

	struct netbuf *inbuf;
	err_t recv_err;
	char *buf;
	u16_t buflen;
	char data[11];
    xQueueMbMqtt_t xQueueMbMqtt;
	/* Read the data from the port, blocking if nothing yet there.We assume the request (the part we care about) is in one netbuf */
	recv_err = netconn_recv(conn, &inbuf);
	if (recv_err == ERR_OK) {
		if (netconn_err(conn) == ERR_OK) {
			netbuf_data(inbuf, (void **) &buf, &buflen);
			xQueueMbMqtt.PortID= 0;
			xQueueMbMqtt.NodeID= *(buf+6);
			xQueueMbMqtt.FunC= *(buf+7);
			xQueueMbMqtt.RegAdr.i8data[0] = *(buf+9);
			xQueueMbMqtt.RegAdr.i8data[1] = *(buf+8);
			xQueueMbMqtt.RegData.i8data[0] = *(buf+11);
			xQueueMbMqtt.RegData.i8data[1] = *(buf+10);
			#define portDEFAULT_WAIT_TIME 1000
			BaseType_t Err = pdFALSE;
			Err = xQueueSend(xQueueDownlinkHandle, &xQueueMbMqtt, portDEFAULT_WAIT_TIME);
			if (Err == pdPASS) {
            }
			else {
				printf("\r\n Send : FALSE \r\n");
			}
		}
		/* End Additional Code for processing Uplink to Desktop app*/
	}
	/* Close the connection (server closes in HTTP) */
	netconn_close(conn);
	/* Delete the buffer (netconn_recv gives us ownership,so we have to make sure to deallocate the buffer) */
	netbuf_delete(inbuf);
}
/*--------------------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief  http server thread
 * @param arg: pointer on argument(not used here)
 * @retval None
 */
void http_server_netconn_thread(void *arg) {

	struct netconn *conn, *newconn;
	err_t err, accept_err;
	printf("\r\n http_server_netconn_thread \r\n");
	xQueueControl_t xQueueControl;
	uint8_t uiSysState;
	xQueueControl.xTask = netHTTPTask;
	do {
		osDelay(10);
		xQueuePeek(xQueueMessageHandle, &uiSysState, 0);
	} while (uiSysState != SYS_HTTP);
	xQueueReceive(xQueueMessageHandle, &uiSysState, 0);
	printf("\r\b HTTP Service startring \r\n");
	/* Create a new TCP connection handle */
	conn = netconn_new(NETCONN_TCP);
	while (1)
		if (conn != NULL) {
			/* Bind to port 502 (Modbus TCP) with default IP address */
			err = netconn_bind(conn, NULL, 80);
			if (err == ERR_OK) {
				/* Put the connection into LISTEN state */
				netconn_listen(conn);
				xQueueControl.xState = TASK_RUNNING;
				xQueueSend(xQueueControlHandle, &xQueueControl, 10);
				while (1) {
					/* accept any icoming connection */
					accept_err = netconn_accept(conn, &newconn);
					if (accept_err == ERR_OK) {
						/* serve connection */
						http_server_serve(newconn);
						/* delete connection */
						netconn_delete(newconn);
					}
				}
			}
		}
}

/*--------------------------------------------------------------------------------------------------------------------------------*/

/**
 * @brief  Initialize the HTTP server (start its thread)
 * @param  none
 * @retval None
 */
void http_server_netconn_init() {
	sys_thread_new("HTTP", http_server_netconn_thread, NULL,DEFAULT_THREAD_STACKSIZE * 12, WEBSERVER_THREAD_PRIO);
	osDelay(10);
}

/*
 * Brief: Handle Time Packet from the Internet.
 * @pram: tcp conn pointer
 * */
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
static void timer_nist_serve(struct netconn *conn) {

	RTC_TimeTypeDef sTime = { 0 };
	RTC_DateTypeDef sDate = { 0 };

	struct netbuf *inbuf;
	err_t recv_err;
	char *buf;
	u16_t buflen;
	char *ptrUser;
	/* Read the data from the port, blocking if nothing yet there.We assume the request (the part we care about) is in one netbuf */
	recv_err = netconn_recv(conn, &inbuf);
	if (recv_err == ERR_OK) {
		if (netconn_err(conn) == ERR_OK) {
			netbuf_data(inbuf, (void **) &buf, &buflen);
			printf("\r\n Received data to client with %d byte:  \n", buflen);
			HAL_UART_Transmit(&huart6, buf, buflen, 1000);
			ptrUser = strstr(buf, "-");
			/*   58954 20-04-15 16:37:07 50 0 0 418.7 UTC(NIST) ** */
			sDate.Month = (*(ptrUser + 1) - 48) * 10 + (*(ptrUser + 2) - 48);
			sDate.Date = (*(ptrUser + 4) - 48) * 10 + (*(ptrUser + 5) - 48);
			sDate.Year = (*(ptrUser - 2) - 48) * 10 + (*(ptrUser - 1) - 48);

			if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
				printf("BUGGGGG");
				Error_Handler();
			}
			ptrUser = strstr(buf, ":");
			#define EDT_TIME_OFFSET +7
			sTime.Hours = (*(ptrUser - 2) - 48) * 10 + (*(ptrUser - 1) - 48 + EDT_TIME_OFFSET);
			sTime.Minutes = (*(ptrUser + 1) - 48) * 10 + (*(ptrUser + 2) - 48);
			sTime.Seconds = (*(ptrUser + 4) - 48) * 10 + (*(ptrUser + 5) - 48);
			if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
				printf("BUGGGGG");
				Error_Handler();
			}
//			printf("\r\n Date: %d %d %d",sDate.Month,sDate.Date,sDate.Year);
			printf("\r\n Time: %d %d %d", sTime.Hours, sTime.Minutes,sTime.Seconds);
			/*Enable chip built-in RTC*/
			__HAL_RCC_RTC_ENABLE();
		} else {
			printf("\r\n Failled to the network time \r\n");
		}
	}
}
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void netFlashSaveTask(void *arg)
{
	xFlashSave();
	vTaskDelay(100);
	NVIC_SystemReset();

	while(1);

}
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void netTcpEcho(struct netconn *pxNetCon) {
	struct netbuf *pxRxBuffer;
	portCHAR *pcRxString;
	unsigned portSHORT usLength;
	struct netbuf *inbuf;
	err_t recv_err;
	char *buf;
	u16_t buflen;
	ip4_addr_t ip;
	uint16_t port;
	char *mess = "conChoLong";
	netconn_write(pxNetCon, mess, (u16_t )15, NETCONN_COPY);
	recv_err = netconn_recv(pxNetCon, &inbuf);
	netconn_getaddr(pxNetCon, &ip, &port, 0);
	mqttHostParam.ip.idata = ip.addr;
	printf("\r\n TCP incomming from: %d %d %d %d", mqttHostParam.ip.cdata[0],
			mqttHostParam.ip.cdata[1], mqttHostParam.ip.cdata[2],
			mqttHostParam.ip.cdata[3]);

	if (recv_err == ERR_OK) {
		if (netconn_err(pxNetCon) == ERR_OK) {
			netbuf_data(inbuf, (void **) &buf, &buflen);
			printf("\r\nReceived data from  to client with %d byte:  \n",
					buflen);
			/*Todo: Give semaphore to reconnect mqtt with new ip*/

			HAL_UART_Transmit(&huart6, buf, buflen, 1000);
			//			netconn_close(pxNetCon);

			/* Delete the buffer (netconn_recv gives us ownership,
			 so we have to make sure to deallocate the buffer) */
			netbuf_delete(inbuf);
		}
	}
	/* Store new MQTT addr - be careful!*/
	osThreadDef(netFlashSave, netFlashSaveTask, osPriorityRealtime, 0, 2 * 128);
	netFlashSave = osThreadCreate(osThread(netFlashSave), NULL);
	__disable_irq();
	xFlashSave();
	__enable_irq();
	portENABLE_INTERRUPTS();

	netconn_close(pxNetCon);
	vTaskDelay(100);
	NVIC_SystemReset();
}
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void NetworkTimeTask(void const *arg) {

	printf("\r\n netTimeTask \r\n");
	struct netconn *conn, *newconn;
	err_t err, accept_err;
	uint8_t TimeNistIP[4];
	/*[132.163.96.3]*/
	TimeNistIP[0] = 132;
	TimeNistIP[1] = 163;
	TimeNistIP[2] = 96;
	TimeNistIP[3] = 3;
	/*Waiting Os for start-up*/
	xQueueControl_t xQueueControl;
	uint8_t uiSysState;
	xQueueControl.xTask = netTimeTask;
	do {
		osDelay(10);
		xQueuePeek(xQueueMessageHandle, &uiSysState, 0);
	}
	while (uiSysState != SYS_NET_TIME);
	xQueueReceive(xQueueMessageHandle, &uiSysState, 0);
	/* Create a new TCP connection handle */
	conn = netconn_new(NETCONN_TCP);
	if (conn != NULL) {
		/* Bind to port 13 with default IP address */
		err = netconn_bind(conn, NULL, 13);
		if (err == ERR_OK) {
			/* Put the connection into LISTEN state */
			netconn_connect(conn, TimeNistIP, 13);
			printf("\r\n Get the network time \r\n");
			timer_nist_serve(conn);
			netconn_delete(conn);
			xQueueControl.xState = TASK_RUNNING;
			xQueueSend(xQueueControlHandle, &xQueueControl, 10);
			while (1) {
				osDelay(1000);
			}
		} else {
			printf("\r\n Failled to the network time \r\n");
		}
	}
	conn = netconn_new(NETCONN_TCP);
}
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void TcpDiscoverTask(void *arg) {
	printf("\r\n netTcpDiscoverTask \r\n");
	struct netconn *conn, *newconn;
	err_t err, accept_err;
	xQueueControl_t xQueueControl;
	uint8_t uiSysState;
	xQueueControl.xTask = netTcpEchoTask;
	do {
		osDelay(10);
		xQueuePeek(xQueueMessageHandle, &uiSysState, 0);
	} while (uiSysState != SYS_CORE_DISCOV);
	xQueueReceive(xQueueMessageHandle, &uiSysState, 0);
	/* Create a new TCP connection handle */
	conn = netconn_new(NETCONN_TCP);
	uint8_t time_delay_core_counter = 0;
#define port_DEFAULT_WAIT_CORE 100
#define port_MAX_WAIT_CORE 250
	if (conn != NULL) {
		err = netconn_bind(conn, NULL, 6969);
		{
			if (err == ERR_OK) {
				netconn_listen(conn);
				while (1) {
					accept_err = netconn_accept(conn, &newconn);
					if (accept_err == ERR_OK) {

						netTcpEcho(newconn);
						netconn_delete(newconn);
						if (time_delay_core_counter < port_DEFAULT_WAIT_CORE) {
							xQueueControl.xState = TASK_RUNNING;
							xQueueSend(xQueueControlHandle, &xQueueControl, 10);
						} else {
							/*Give Semaphore for mqtt stack restart mqtt server*/
							//xSemaphoreGive(netMqttIpSemaphoreHandle);
//								netMqttIpSemaphoreHandle
						}
					}
					vTaskDelay(100);
					if (time_delay_core_counter < port_DEFAULT_WAIT_CORE) {
						time_delay_core_counter++;
					}

					if ((time_delay_core_counter > port_DEFAULT_WAIT_CORE)
							&& (time_delay_core_counter < port_MAX_WAIT_CORE)) {
						time_delay_core_counter = port_MAX_WAIT_CORE;
						xQueueControl.xState = TASK_ERR_1;
						xQueueSend(xQueueControlHandle, &xQueueControl, 10);
					}
				}
			}
		}
	}
}

