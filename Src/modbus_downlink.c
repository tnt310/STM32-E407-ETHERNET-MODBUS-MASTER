/* Built-in C library includes ---------------*/
#include <param.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "sharedmem.h"
//#include "mb.h"
#include "mb_m.h"
#include "mbframe.h"
#include "mbport.h"
#include "mbconfig.h"
#include "param.h"

extern osMessageQId xQueueControlHandle;
extern osMessageQId xQueueMessageHandle;
extern osMessageQId xQueueDownlinkHandle;
extern osMessageQId xQueueUplinkHandle;
extern osThreadId mbDownlinkTask;
extern volatile uint8_t read_mutex;
extern volatile uint8_t write_mutex;
///* Private variables ---------------------------------------------------------*/
//


void ModbusDownlinkTask(void const *argument) {

	osDelay(100);
		printf("\r\n ModbusDownlinkTask \r\n");
		BaseType_t xError;
		xQueueControl_t xQueueControl;
		uint8_t uiSysState;
		xQueueControl.xTask = mbDownlinkTask;
		do {
			osDelay(10);
			xQueuePeek(xQueueMessageHandle, &uiSysState, 0);
		} while (uiSysState != SYS_MB_DOWNLINK);
		xQueueReceive(xQueueMessageHandle, &uiSysState, 0);
		printf("\r\n ModbusDownlinkTask: Starting");
		#define portDEFAULT_WAIT_TIME 1000
		BaseType_t Err = pdFALSE;
		xQueueMbMqtt_t xQueueMbMqtt;
		osDelay(500);
		eMBErrorCode eStatus = MB_ENOERR;
		xQueueControl.xState = TASK_RUNNING;
		xQueueSend(xQueueControlHandle, &xQueueControl, 10);
		#define MB_DEFAULT_TEST_NREG	0x01
		#define MB_DEFAULT_TEST_TIMEOUT  1
			while (1) {
				Err = xQueueReceive(xQueueDownlinkHandle, &xQueueMbMqtt,portDEFAULT_WAIT_TIME);
				if (Err == pdPASS) {
					//printf("\r\n DA NHAN DUOC \r\n");
					switch (xQueueMbMqtt.FunC) {
					case MB_FUNC_READ_HOLDING_REGISTER:
						read_mutex = 1;
						eMBMasterReqReadHoldingRegister(xQueueMbMqtt.PortID,xQueueMbMqtt.NodeID, xQueueMbMqtt.RegAdr.i16data,xQueueMbMqtt.RegData.i16data, MB_DEFAULT_TEST_TIMEOUT);
						break;
					case MB_FUNC_WRITE_REGISTER:
						write_mutex = 1;
						eMBMasterReqWriteHoldingRegister(xQueueMbMqtt.PortID,xQueueMbMqtt.NodeID, xQueueMbMqtt.RegAdr.i16data,xQueueMbMqtt.RegData.i16data, MB_DEFAULT_TEST_NREG);
						break;
					case MB_FUNC_WRITE_SINGLE_COIL:
						eMBMasterReqWriteCoil(xQueueMbMqtt.PortID,
													xQueueMbMqtt.NodeID, xQueueMbMqtt.RegAdr.i16data,
													xQueueMbMqtt.RegData.i16data, MB_DEFAULT_TEST_TIMEOUT);
						break;
					case MB_FUNC_READ_COILS:
						eMBMasterReqReadCoils(xQueueMbMqtt.PortID,
											            xQueueMbMqtt.NodeID, xQueueMbMqtt.RegAdr.i16data,
														xQueueMbMqtt.RegData.i16data , MB_DEFAULT_TEST_TIMEOUT);
					}

				}
			}
}
