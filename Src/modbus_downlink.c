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
///* Private variables ---------------------------------------------------------*/
//
#define M_REG_HOLDING_START            1
#define M_REG_HOLDING_NREGS           200

#define M_REG_INPUT_START              1
#define M_REG_INPUT_NREGS             200

#define M_REG_COIL_START               1
#define M_REG_COIL_NREGS               200

#define M_REG_DISCRETE_START           1
#define M_REG_DISCRETE_NREGS           200

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
					switch (xQueueMbMqtt.FunC) {
					case MB_FUNC_WRITE_REGISTER:
						eMBMasterReqWriteHoldingRegister(xQueueMbMqtt.PortID,
														 xQueueMbMqtt.NodeID, xQueueMbMqtt.RegAdr.i16data,
														 xQueueMbMqtt.RegData.i16data, MB_DEFAULT_TEST_NREG);
						break;
					}

				}
			}
}
///************************Master Callback Function for Holding Register****************************************************************************/
eMBErrorCode eMBMasterRegHoldingCB_v1(UCHAR ucPort, UCHAR * pucRegBuffer, USHORT usAddress,USHORT usNRegs, eMBRegisterMode eMode)
{
	eMBErrorCode eStatus = MB_ENOERR;
	USHORT iRegIndex;
	USHORT REG_HOLDING_START = M_REG_HOLDING_START;
	USHORT REG_HOLDING_NREGS = M_REG_HOLDING_NREGS;
	/* FreeRTOS variable*/
	xQueueMbMqtt_t xQueueMbMqtt;
	xQueueMbMqtt.PortID = ucPort;
	xQueueMbMqtt.NodeID = ucMBMasterGetDestAddress(ucPort);
	/* if mode is read, the master will write the received date to buffer. */
	usAddress--;
	xQueueMbMqtt.RegAdr.i8data[0] = (uint8_t)usAddress;
	xQueueMbMqtt.RegAdr.i8data[1] = (uint8_t)(usAddress >>8);
	if ((usAddress >= REG_HOLDING_START)&& ((uint8_t)usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS)) {
		iRegIndex = usAddress - REG_HOLDING_START;
		switch (eMode) {
		case MB_REG_WRITE:
			xQueueMbMqtt.FunC = MB_FUNC_WRITE_REGISTER;
			while (usNRegs > 0)
			{
				xQueueMbMqtt.RegData.i8data[1] = (*pucRegBuffer);
				xQueueMbMqtt.RegData.i8data[0] = *(pucRegBuffer + 1);
				iRegIndex++;
				usNRegs--;
				printf("\r\n  data: %d ",xQueueMbMqtt.RegData.i16data);
				printf("\r\n*****************\r\n");
			}
			break;
		}
		BaseType_t Err = pdFALSE;
		Err = xQueueSend(xQueueUplinkHandle, &xQueueMbMqtt,portDEFAULT_WAIT_TIME);
		if (Err == pdPASS) {
			} else {
			printf("\r\n Modbus_MQTT Up queued: False \r\n");
		}
		} else {
		eStatus = MB_ENOREG;
	}
	return eStatus;
}
