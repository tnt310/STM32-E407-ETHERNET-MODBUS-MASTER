#ifndef	_COMMAND_H
#define	_COMMAND_H
#include "cmdline.h"
#include <stdint.h>
#include <stdbool.h>

extern uint32_t gotCommandFlag;
extern uint8_t commandBuffer[200];
int
Cmd_help(int argc, char *argv[]);
int
setRGBLED(int argc, char *argv[]);
int
setLight(int argc, char *argv[]);
int
controlRelay(int argc, char *argv[]);
int
setAllLight(int argc, char *argv[]);
void UARTIntHandler(void);
uint8_t LoadSdcard(char *file);
uint8_t RecordData(char *file, char *buffer);
uint8_t CheckRecord(char *file);
void ftoa(char buffer[20], char string[20], uint16_t scale);
uint8_t FloatToString(char buffer[20], uint32_t float_value);
uint8_t Load_deviceStatus(uint8_t port,uint8_t deviceID,uint8_t func,uint16_t deviceChannel,char *deviceType,char *deviceName,char *deviceTitle,char *valueType, char *regtype, uint16_t scale, uint8_t devicestatus);
#endif
