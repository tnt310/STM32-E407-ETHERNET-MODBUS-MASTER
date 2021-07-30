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
#include "flash.h"
#include "command.h"
#include "sdcard.h"

/* Shared Variable ----------------------------------*/
osThreadId mbProtocolTask;
osThreadId mbAppTask;
osThreadId mbDownlinkTask;

extern osMessageQId xQueueDownlinkHandle;
extern osMessageQId xQueueUplinkHandle;
extern TIM_HandleTypeDef htim7;
extern osTimerId myTimer01Handle;
extern osMessageQId xQueueControlHandle;
extern osMessageQId xQueueMessageHandle;

data1_t *dynamic;
uint8_t num_device;
extern uint32_t modbus_mutex;
extern uint32_t modbus_telemetry;
extern uint32_t timeDelay;
volatile uint8_t read_mutex;
volatile uint8_t write_mutex;
char buffer[20];
char err_buffer[50];
uint8_t active, negative, float_t, flag = 0;
extern size_t packet, error;
/* Private variables ---------------------------------------------------------*/

#define M_REG_HOLDING_START            0
#define M_REG_HOLDING_NREGS            65000

#define M_REG_COIL_START               0
#define M_REG_COIL_NREGS               65000

#define M_REG_INPUT_START              0
#define M_REG_INPUT_NREGS              65000
/*----------------------------------------------------------------------------------------------------------------------------------*/

void ModbusRTUTask(void const *argument) {
	#define PORT_INF_DELAY 0
	osDelay(150);
	printf("\r\n ModbusRTUTask \r\n");
	BaseType_t xError;
	xQueueControl_t xQueueControl;
	uint8_t uiSysState;
	xQueueControl.xTask = mbProtocolTask;
	do {
		osDelay(10);
		xQueuePeek(xQueueMessageHandle, &uiSysState, 0);
	} while (uiSysState != SYS_MB_PROTOCOL);
	xQueueReceive(xQueueMessageHandle, &uiSysState, 0);
	printf("\r\n ModbusRTUTask Initing");
	eMBErrorCode eStatus = eMBMasterInit(MB_RTU, 1, 9600, MB_PAR_NONE);
	eStatus = eMBMasterEnable(PORT1);
	eStatus = eMBMasterEnable(PORT2);
	HAL_TIM_Base_Start_IT(&htim7);
	/*State control machine*/
	xQueueControl.xState = TASK_RUNNING;
	xQueueSend(xQueueControlHandle, &xQueueControl, 10);
	while (1) {
		eMBMasterPoll();
		vTaskDelay(1);
	}
}
/************************************************************************************************************************************-*/

void ModbusTestTask(void const *argument) {

	osDelay(100);
	printf("\r\n ModbusTestTask \r\n");
	BaseType_t xError;
	xQueueControl_t xQueueControl;
	uint8_t uiSysState;
	xQueueControl.xTask = mbAppTask;
	do {
		osDelay(10);
		xQueuePeek(xQueueMessageHandle, &uiSysState, 0);
	} while (uiSysState != SYS_MB_APP);
	xQueueReceive(xQueueMessageHandle, &uiSysState, 0);
	printf("\r\n ModbusTestTask: Starting");
	#define portDEFAULT_WAIT_TIME 1000
	BaseType_t Err = pdFALSE;
	xQueueMbMqtt_t xQueueMbMqtt;
	osDelay(500);
	eMBErrorCode eStatus = MB_ENOERR;
	xQueueControl.xState = TASK_RUNNING;
	xQueueSend(xQueueControlHandle, &xQueueControl, 10);
	#define MB_DEFAULT_TEST_NREG	0x01
	#define MB_DEFAULT_TEST_TIMEOUT  100
	device_t device;
	uint8_t count = 0;
	while (1) {
		while(modbus_telemetry){
			HAL_Delay(1000);
					for (uint8_t i = 0;i < num_device ; i++){
						for (uint8_t j = 0; j < num_device ; j++){
							if ((dynamic + j)->deviceID == (dynamic +i)->deviceID && (dynamic +i)->deviceID != (dynamic +i-1)->deviceID){
				                count++;
				                if (count == 1)
				                    {
				                        if (i > 0){
				                        for (uint8_t a = 0; a < i; a++){
				                             if ((dynamic +a)->deviceID == (dynamic +j)->deviceID){
				                                goto TEST;
				                                }
				                            }
				                        }
				                        for (uint8_t z = 0; z < num_device ; z++)
				                        {
				                            if ((dynamic +z)->deviceID == (dynamic +j)->deviceID ){
				                            	device.channel = (dynamic +z)->channel;
				                            	device.id = (dynamic +z)->deviceID;
				                            	device.func = (dynamic +z)->func;
				                            	device.regAdr = (dynamic +z)->deviceChannel;
				                            	device.numreg =(dynamic +z)->numreg;
				                            	device.status = (dynamic +z)->devicestatus;
				                            	flag = 1;
				                            	if (device.status == 1){
					                            	switch(device.channel)
					                            	{
					                            		case 1:
					                            		{
															switch(device.func)
															{
																case MB_FUNC_READ_HOLDING_REGISTER:
																	if (eMBMasterReqReadHoldingRegister(device.channel, device.id, device.regAdr,device.numreg, MB_DEFAULT_TEST_TIMEOUT) ==MB_MRE_NO_ERR){
																		packet ++;
																	}else{
																		SD_ErrorPacket(err_buffer, device.channel, device.id, device.regAdr);
																		write_sdcard("event.txt",err_buffer);
																		error ++;
																	}
																	break;
																case MB_FUNC_READ_COILS:
																	eMBMasterReqReadCoils(device.channel, device.id, device.regAdr,device.numreg, MB_DEFAULT_TEST_TIMEOUT);
																	break;
																case MB_FUNC_READ_INPUT_REGISTER:
																	eMBMasterReqReadInputRegister(device.channel, device.id, device.regAdr,device.numreg, MB_DEFAULT_TEST_TIMEOUT);
																	//break;
															}
					                            		}
					                            		break;
					                            		case 0:
					                            		{
															switch(device.func)
															{
																case MB_FUNC_READ_HOLDING_REGISTER:
																	if (eMBMasterReqReadHoldingRegister(device.channel, device.id, device.regAdr,device.numreg, MB_DEFAULT_TEST_TIMEOUT) ==MB_MRE_NO_ERR){
																		packet ++;
																	}else{
																		SD_ErrorPacket(err_buffer, device.channel, device.id, device.regAdr);
																		write_sdcard("event.txt",err_buffer);
																		error ++;
																	}
																	break;
																case MB_FUNC_READ_COILS:
																	eMBMasterReqReadCoils(device.channel, device.id, device.regAdr,device.numreg, MB_DEFAULT_TEST_TIMEOUT);
																	break;
																case MB_FUNC_READ_INPUT_REGISTER:
																	eMBMasterReqReadInputRegister(device.channel, device.id, device.regAdr,device.numreg, MB_DEFAULT_TEST_TIMEOUT);
																	//break;
															}
															break;
					                            		}
					                            	}
				                            	}
				                            	HAL_Delay(100);
				                            }
				                        }
				                    }
							}
						}
						TEST: count = 0;
					}
					if (flag == 1){
						HAL_Delay(100);
						xQueueMbMqtt.gotflagLast = 2;
						BaseType_t Err = pdFALSE;
						Err = xQueueSend(xQueueUplinkHandle, &xQueueMbMqtt,portDEFAULT_WAIT_TIME);
						if (Err == pdPASS){
							xQueueMbMqtt.gotflagLast = 0;
						}
						HAL_Delay((timeDelay * 1000)- (200 * num_device)-100- 10000);
					}
			}
	}
}
/*--------------------------Master Callback Function for Holding Register---------------------------------------------------------------------------*/
eMBErrorCode eMBMasterRegHoldingCB(UCHAR ucPort, UCHAR * pucRegBuffer, USHORT usAddress,USHORT usNRegs, eMBRegisterMode eMode)
{
	USHORT usMRegHoldStart = M_REG_HOLDING_START;
	USHORT usMRegHoldBuf[MB_RS485_MAX_PORT][MB_MASTER_TOTAL_SLAVE_NUM][M_REG_HOLDING_NREGS];
	eMBErrorCode eStatus = MB_ENOERR;
	USHORT iRegIndex;
	USHORT * pusRegHoldingBuf;
	USHORT REG_HOLDING_START;
	USHORT REG_HOLDING_NREGS;
	USHORT usRegHoldStart;
	REG_HOLDING_START = M_REG_HOLDING_START;
	REG_HOLDING_NREGS = M_REG_HOLDING_NREGS;
	/* FreeRTOS variable*/
	xQueueMbMqtt_t xQueueMbMqtt;
	xQueueMbMqtt.PortID = ucPort;
	xQueueMbMqtt.NodeID = ucMBMasterGetDestAddress(ucPort);
	/* if mode is read, the master will write the received date to buffer. */
	usAddress--; // must have if u do not want to be stupid
	xQueueMbMqtt.RegAdr.i8data[0] = (uint8_t)usAddress;
	xQueueMbMqtt.RegAdr.i8data[1] = (uint8_t)(usAddress >> 8);
	uint8_t reg_temp = usNRegs;
	if (read_mutex == 1 || write_mutex ==1){
		goto COMMAND;
	}
	for (uint8_t i = 0; i < num_device; i++){
		if ((dynamic+i)->channel == xQueueMbMqtt.PortID  && (dynamic+i)->deviceID == xQueueMbMqtt.NodeID && (dynamic+i)->deviceChannel == xQueueMbMqtt.RegAdr.i16data){
				memset(buffer,'\0',sizeof(buffer));
				strncpy(buffer, (dynamic+i)->regtype, strlen((dynamic+i)->regtype));
				}
			}
	if (strstr(buffer,"UINT16") != NULL || strstr(buffer,"UINT32") != NULL || strstr(buffer,"UINT64") != NULL){
		active = 1;
	}else if (strstr(buffer,"INT16") != NULL || strstr(buffer,"INT32") != NULL || strstr(buffer,"INT64") != NULL){
		negative = 1;
	}else if(strstr(buffer,"FLOAT32") != NULL){
		float_t = 1;
	}
	if ((usAddress >= REG_HOLDING_START)&& ((uint8_t)usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS)) {
		iRegIndex = usAddress - REG_HOLDING_START;
		switch (eMode) {
		case MB_REG_READ:
			xQueueMbMqtt.FunC = MB_FUNC_READ_HOLDING_REGISTER;
			while (usNRegs > 0)
			{
				if (reg_temp == 1) { // with U16, I16
					if (active == 1){
						active = 0;
						xQueueMbMqtt.RegData.i8data[1] = *(pucRegBuffer);  // byte 1
						xQueueMbMqtt.RegData.i8data[0] = *(pucRegBuffer + 1);// byte 0
						pusRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
						pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
						iRegIndex++;
						usNRegs--;
						xQueueMbMqtt.gotflagvalue = 0;
						//printf("\r\n HOLDING REGISTER CALLBACK AT PORT %d: %d %d \r\n", xQueueMbMqtt.PortID,xQueueMbMqtt.RegAdr.i16data,xQueueMbMqtt.RegData.i16data);
					}else if (negative == 1){
						negative = 0;
						xQueueMbMqtt.IRegData.i8data[1] = *(pucRegBuffer);  // byte 1
						xQueueMbMqtt.IRegData.i8data[0] = *(pucRegBuffer + 1);// byte 0
						pusRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
						pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
						iRegIndex++;
						usNRegs--;
						xQueueMbMqtt.gotflagvalue = 1;
						//printf("\r\n HOLDING REGISTER CALLBACK AT PORT %d: %d %d \r\n", xQueueMbMqtt.PortID,xQueueMbMqtt.RegAdr.i16data,xQueueMbMqtt.IRegData.i16data);
					}
				}
				else if (reg_temp == 2){ // with U32, I32, FLOAT32
					if (active == 1){
						active = 0;
						xQueueMbMqtt.RegData32.i8data[1] = *(pucRegBuffer);  // byte 1
						xQueueMbMqtt.RegData32.i8data[0] = *(pucRegBuffer + 1);// byte 0
						pusRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
						pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
						iRegIndex++;
						usNRegs--;
						xQueueMbMqtt.RegData32.i8data[3] = *(pucRegBuffer);  // byte 1
						xQueueMbMqtt.RegData32.i8data[2] = *(pucRegBuffer + 1);// byte 0
						pusRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
						pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
						iRegIndex++;
						usNRegs--;
						xQueueMbMqtt.gotflagvalue = 0;

					}else if (negative == 1){
						negative = 0;
						xQueueMbMqtt.IRegData32.i8data[1] = *(pucRegBuffer);  // byte 1
						xQueueMbMqtt.IRegData32.i8data[0] = *(pucRegBuffer + 1);// byte 0
						pusRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
						pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
						iRegIndex++;
						usNRegs--;
						xQueueMbMqtt.IRegData32.i8data[3] = *(pucRegBuffer);  // byte 1
						xQueueMbMqtt.IRegData32.i8data[2] = *(pucRegBuffer + 1);// byte 0
						pusRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
						pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
						iRegIndex++;
						usNRegs--;
						xQueueMbMqtt.gotflagvalue = 1;
					}else if (float_t == 1){
						float_t = 0;
						xQueueMbMqtt.RegData32.i8data[1] = *(pucRegBuffer);  // byte 1
						xQueueMbMqtt.RegData32.i8data[0] = *(pucRegBuffer + 1);// byte 0
						pusRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
						pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
						iRegIndex++;
						usNRegs--;
						xQueueMbMqtt.RegData32.i8data[3] = *(pucRegBuffer);  // byte 1
						xQueueMbMqtt.RegData32.i8data[2] = *(pucRegBuffer + 1);// byte 0
						pusRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
						pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
						iRegIndex++;
						usNRegs--;
						xQueueMbMqtt.gotflagvalue = 2;
						//printf("\r\n HOLDING REGISTER CALLBACK AT PORT %d: %d %d \r\n", xQueueMbMqtt.PortID,xQueueMbMqtt.RegAdr.i16data,xQueueMbMqtt.RegData.i16data);
					}
					xQueueMbMqtt.flag32 = 1;
				}
			}
			if (read_mutex == 1){
				goto MUTEX;
			}
			break;
		case MB_REG_WRITE:
			xQueueMbMqtt.FunC = MB_FUNC_WRITE_REGISTER;
			while (usNRegs > 0) {
				xQueueMbMqtt.RegData.i8data[1] = (*pucRegBuffer);
				xQueueMbMqtt.RegData.i8data[0] = *(pucRegBuffer + 1);
				*pucRegBuffer++ = (UCHAR) (pusRegHoldingBuf[iRegIndex] >> 8);
				*pucRegBuffer++ = (UCHAR) (pusRegHoldingBuf[iRegIndex] & 0xFF);
				iRegIndex++;
				usNRegs--;
			}
			if (write_mutex == 1){
				goto MUTEX;
			}
			break;
		}
		xQueueMbMqtt.gotflagtelemetry = 2;
 MUTEX:	for (uint8_t i = 0; i < num_device; i++){
			if ((dynamic+i)->deviceID == xQueueMbMqtt.NodeID && (dynamic+i)->deviceChannel == xQueueMbMqtt.RegAdr.i16data && (dynamic+i)->func == xQueueMbMqtt.FunC ){
				xQueueMbMqtt.scale = (dynamic+i)->scale;
			}
		}
 COMMAND: if (read_mutex==1 || write_mutex==1){
 		  if ((usAddress >= REG_HOLDING_START)&& ((uint8_t)usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS)) {
 			iRegIndex = usAddress - usRegHoldStart;
 			switch (eMode){
 			case MB_REG_WRITE:
 				xQueueMbMqtt.FunC = MB_FUNC_WRITE_REGISTER;
 				while (usNRegs > 0) {
 					xQueueMbMqtt.RegData.i8data[1] = (*pucRegBuffer);
 					xQueueMbMqtt.RegData.i8data[0] = *(pucRegBuffer + 1);
 					*pucRegBuffer++ = (UCHAR) (pusRegHoldingBuf[iRegIndex] >> 8);
 					*pucRegBuffer++ = (UCHAR) (pusRegHoldingBuf[iRegIndex] & 0xFF);
 					iRegIndex++;
 					usNRegs--;
 				}
 				break;
 			case MB_REG_READ:
 				xQueueMbMqtt.FunC = MB_FUNC_READ_HOLDING_REGISTER;
 				while (usNRegs > 0) {
 					if (reg_temp == 2){
 						xQueueMbMqtt.RegData32.i8data[1] = *(pucRegBuffer);  // byte 1
 						xQueueMbMqtt.RegData32.i8data[0] = *(pucRegBuffer + 1);// byte 0
 						pusRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
 						pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
 						iRegIndex++;
 						usNRegs--;
 						xQueueMbMqtt.RegData32.i8data[3] = *(pucRegBuffer);  // byte 1
 						xQueueMbMqtt.RegData32.i8data[2] = *(pucRegBuffer + 1);// byte 0
 						pusRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
 						pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
 						iRegIndex++;
 						usNRegs--;
 						xQueueMbMqtt.gotflagvalue = 0;
 						xQueueMbMqtt.flag32 = 1;
 						printf("\r\n U32 on CB : %d \r\n",xQueueMbMqtt.RegData32.i32data);
 					}else {
 						xQueueMbMqtt.RegData.i8data[1] = *(pucRegBuffer);  // byte 1
 						xQueueMbMqtt.RegData.i8data[0] = *(pucRegBuffer + 1);// byte 0
 						pusRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
 						pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
 						iRegIndex++;
 						usNRegs--;
 						xQueueMbMqtt.gotflagvalue = 0;
 						//printf("\r\n U16 on CB : %d\r\n",xQueueMbMqtt.RegData.i16data);
 					}
 				}
 				break;
 			}
 		}
 	}
		if (write_mutex == 1 || read_mutex == 1){
	 	 	 read_mutex = 0;
	 	 	 write_mutex = 0;
	 	 	 xQueueMbMqtt.scale = 1;
			xQueueMbMqtt.gotflagcommand = 3;
		}
		BaseType_t Err = pdFALSE;
		Err = xQueueSend(xQueueUplinkHandle, &xQueueMbMqtt,portDEFAULT_WAIT_TIME);
		if (Err == pdPASS) {
			xQueueMbMqtt.gotflagtelemetry = 0;
			xQueueMbMqtt.gotflagvalue = 0;
			} else {
			printf("\r\n Modbus_MQTT Up queued: False \r\n");
		}
	}
	else {
		eStatus = MB_ENOREG;
	}
	return eStatus;
}
/************************Master Callback Function for Input Register****************/
eMBErrorCode eMBMasterRegInputCB(UCHAR ucPort, UCHAR * pucRegBuffer, USHORT usAddress,USHORT usNRegs)
{
	USHORT usMRegHoldStart = M_REG_INPUT_START;
	USHORT usMRegHoldBuf[MB_RS485_MAX_PORT][MB_MASTER_TOTAL_SLAVE_NUM][M_REG_INPUT_NREGS];
	eMBErrorCode eStatus = MB_ENOERR;
	USHORT iRegIndex;
	USHORT * pusRegHoldingBuf;
	USHORT REG_INPUT_START;
	USHORT REG_INPUT_NREGS;
	USHORT usRegHoldStart;
	REG_INPUT_START= M_REG_INPUT_START;
	REG_INPUT_NREGS = M_REG_INPUT_NREGS;
	/* FreeRTOS variable*/
		xQueueMbMqtt_t xQueueMbMqtt;
		xQueueMbMqtt.PortID = ucPort;
		xQueueMbMqtt.NodeID = ucMBMasterGetDestAddress(ucPort);
	/* if mode is read, the master will write the received date to buffer. */
		usAddress--;
		xQueueMbMqtt.RegAdr.i8data[0] = (uint8_t)usAddress;
		xQueueMbMqtt.RegAdr.i8data[1] = (uint8_t)(usAddress >>8);
		uint8_t reg_temp = usNRegs;
		if ((usAddress >= REG_INPUT_START) && ((uint8_t)usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
		  {
			iRegIndex =  usAddress - REG_INPUT_NREGS;
			xQueueMbMqtt.FunC = MB_FUNC_READ_INPUT_REGISTER;
			while (usNRegs > 0)
					{
				    	xQueueMbMqtt.RegData.i8data[1] = *(pucRegBuffer);  // byte 1
				    	xQueueMbMqtt.RegData.i8data[0] = *(pucRegBuffer + 1);// byte 0
				    	pusRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
				    	pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
				    	iRegIndex++;
				    	usNRegs--;
				    	printf("\r\n INPUT REGISTER CALLBACK AT PORT %d: %d \r\n", xQueueMbMqtt.PortID,xQueueMbMqtt.RegData.i16data);
					}
//			for (uint8_t i = 0; i < num_device; i++){
//				if ((dynamic+i)->deviceID == xQueueMbMqtt.NodeID && (dynamic+i)->deviceChannel == xQueueMbMqtt.RegAdr.i16data && (dynamic+i)->func == xQueueMbMqtt.FunC ){
//					xQueueMbMqtt.scale = (dynamic+i)->scale;
//				}
//			}
//			xQueueMbMqtt.gotflagtelemetry = 2; // update count for device
//			BaseType_t Err = pdFALSE;
//			Err = xQueueSend(xQueueUplinkHandle, &xQueueMbMqtt,portDEFAULT_WAIT_TIME);
//			if (Err == pdPASS) {
//				xQueueMbMqtt.gotflagtelemetry = 0;
//				} else {
//				printf("\r\n Modbus_MQTT Up queued: False \r\n");
//			}
		} else {
			eStatus = MB_ENOREG;
		}
		return eStatus;
}
/*--------------------------------------------------------------------------------------------------------------------------------------------*/
eMBErrorCode eMBMasterRegCoilsCB( UCHAR ucPort,  UCHAR * pucRegBuffer, USHORT usAddress,USHORT usNCoils, eMBRegisterMode eMode ){

				eMBErrorCode eStatus = MB_ENOERR;
				USHORT iRegIndex;
				USHORT REG_COIL_START = M_REG_COIL_START;
				USHORT REG_COIL_NREGS = M_REG_COIL_NREGS;
			/* FreeRTOS variable*/
				xQueueMbMqtt_t xQueueMbMqtt;
				xQueueMbMqtt.PortID = ucPort;
				xQueueMbMqtt.NodeID = ucMBMasterGetDestAddress(ucPort);
				/* if mode is read, the master will write the received date to buffer. */
				usAddress--;
				xQueueMbMqtt.RegAdr.i8data[0] = (uint8_t)usAddress;
				xQueueMbMqtt.RegAdr.i8data[1] = (uint8_t)(usAddress >>8);
				if ((usAddress >= REG_COIL_START)&& ((uint8_t)usAddress + usNCoils <= REG_COIL_START + REG_COIL_NREGS)) {
					iRegIndex = usAddress - REG_COIL_START;
					switch (eMode) {
					case MB_REG_WRITE:
						xQueueMbMqtt.FunC = MB_FUNC_WRITE_SINGLE_COIL;
						while (usNCoils > 0)
						{
							xQueueMbMqtt.RegData.i8data[1] = (*pucRegBuffer);
							xQueueMbMqtt.RegData.i8data[0] = *(pucRegBuffer + 1);
							iRegIndex++;
							usNCoils--;
						}
						break;
					case MB_REG_READ:
						xQueueMbMqtt.FunC = MB_FUNC_READ_COILS;
						while (usNCoils> 0)
						{
							printf("\r\n  COIL REGISTER CALLBACK AT PORT %d: %d \r\n", xQueueMbMqtt.PortID,*(pucRegBuffer));
							//printf("\r\n  COIL REGISTER CALLBACK: %d \r\n ", *(pucRegBuffer));
							//printf("\r\nRecived data: %d  ", (*pucRegBuffer));
							xQueueMbMqtt.RegData.i8data[1] = (*pucRegBuffer);
							//xQueueMbMqtt.RegData.i8data[0] = *(pucRegBuffer+ 1);
							iRegIndex++;
							usNCoils--;
						}
						break;
					}
				} else {
					eStatus = MB_ENOREG;
				}
				return eStatus;

}
