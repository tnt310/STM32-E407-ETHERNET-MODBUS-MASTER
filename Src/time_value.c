#include "time_value.h"

ip4_addr_t server_addr;

uint8_t getTime(uint8_t time[6])
{
	RTC_TimeTypeDef sTime = { 0 };
	RTC_DateTypeDef sDate = { 0 };
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	time[0] = sDate.Year;
	time[1] = sDate.Month;
	time[2] = sDate.Date;
	time[3] = sTime.Hours;
	time[4] = sTime.Minutes;
	time[5] = sTime.Seconds;
}
