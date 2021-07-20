
#include <string.h>
#include "param.h"
#include <stdint.h>
#include <stdio.h>
#include <telemetry.h>
#include "mqtt.h"
#include "mqttclienttask.h"

uint8_t ping_json(char *buffer, uint8_t id_gateway)
{
    memset(buffer,0,sizeof(buffer));
    sprintf(buffer,"{\"gateway\":%d,\"message\":\"HELLO SERVER\"}",id_gateway);
}
/*---------------------------------------------------------------------------------------------------------------------------------------*/
uint8_t  head_provision(char *buffer,uint8_t device_id, char *device_name, char *device_type)
{
    memset(buffer,'\0',sizeof(buffer));
    sprintf(buffer,"{\"data\":[{\"device_id\":\"%d\",\"device_name\":\"%s\",\"device_kind\":\"%s\",\"device_channels\":[",device_id,device_name,device_type);
}
uint8_t  tail_provision(char *buffer,uint16_t channel_id, char *channel_name, char *channel_type, uint8_t function)
{
    memset(buffer,'\0',sizeof(buffer));
    sprintf(buffer,"{\"channel_id\":\"%d\",\"channel_name\":\"%s\",\"channel_type\":\"%s\",\"channel_commands\":{",channel_id,channel_name,channel_type);
	if(function == 3){
		strcat(buffer,"\"READ\":[]}");
	}
	if(function == 6){
		strcat(buffer,"\"WRITE\":[]}");
	}
    strcat(buffer,"},");
}
/*---------------------------------------------------------------------------------------------------------------------------------------*/
//uint8_t head_telemetry(char *buffer, uint8_t time[6], uint8_t device_id)
//{
//    memset(buffer,'\0',sizeof(buffer));
//    sprintf(buffer,"{\"timestamp\":\"%d-%d-%dT%d:%d:%d.000Z\",\"data\":{\"%d\":",time[0],time[1],time[2],time[3],time[4],time[5],device_id);
//}
uint8_t timestamp_telemetry(char *buffer, uint8_t time[6])
{
    memset(buffer,'\0',sizeof(buffer));
    char month[3];
    char day[3];
    char hour[3];
    char minute[3];
    char second[3];
    if (time[1] < 10){
    	sprintf(month,"0%d",time[1]);
    }else
    	sprintf(month,"%d",time[1]);
    if (time[2] < 10){
    	sprintf(day,"0%d",time[2]);
    }else
    	sprintf(day,"%d",time[2]);
    if (time[3] < 10){
    	sprintf(hour,"0%d",time[3]);
    }else
    	sprintf(hour,"%d",time[3]);
    if (time[4] < 10){
    	 sprintf(minute,"0%d",time[4]);
    }else
    	 sprintf(minute,"%d",time[4]);
    if (time[5] < 10){
    	 sprintf(second,"0%d",time[5]);
    }else
    	 sprintf(second,"%d",time[5]);
    sprintf(buffer,"{\"timestamp\":\"20%d-%s-%sT%s:%s:%s.000Z\",\"data\":{",time[0],month,day,hour,minute,second);
}
//uint8_t head_telemetry(char *buffer, uint8_t time[6], uint8_t device_id)
//{
//    memset(buffer,'\0',sizeof(buffer));
//    sprintf(buffer,"{\"timestamp\":\"20%d-%d-%dT%d:%d:%d.000Z\",\"data\":{\"%d\":",time[0],time[1],time[2],time[3],time[4],time[5],device_id);
//}
uint8_t tail_telemetry(char *buffer, uint16_t device_channel, char *value) //
{
    memset(buffer,'\0',sizeof(buffer));
    sprintf(buffer,"\"%d\":%s,",device_channel, value);
}

uint8_t head_telemetry(char *buffer, uint8_t device_id)
{
    memset(buffer,'\0',sizeof(buffer));
    sprintf(buffer,"\"%d\":{",device_id);
}
//uint8_t tail_telemetry(char *buffer, uint16_t device_channel, uint16_t value) //
//{
//    memset(buffer,'\0',sizeof(buffer));
//    sprintf(buffer,"\"%d\":%d,",device_channel, value);
//}
/*---------------------------------------------------------------------------------------------------------------------------------------*/
uint8_t command_read_json(char *buffer,uint8_t device_id, uint16_t device_channel,char *value)
{
    memset(buffer,0,sizeof(buffer));
    sprintf(buffer,"{\"data\":{\"%d\":{\"%d\":%s}}}",device_id,device_channel,value);
}
uint8_t command_write_json(char *buffer,uint8_t device_id, uint16_t device_channel)
{
    memset(buffer,0,sizeof(buffer));
    sprintf(buffer,"{\"data\":{\"%d\":{\"%d\":{\"command\":\"done\"}}}}",device_id,device_channel);
}
/*---------------------------------------------------------------------------------------------------------------------------------------*/
uint8_t topic(char *buffer, char *apikey, char *topic, char *flow_type)
{
	memset(buffer,'\0',sizeof(buffer));
	sprintf(buffer,"%s/%s/%s",flow_type,topic,apikey);
	}



