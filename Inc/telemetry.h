/*
 * telemetry.h
 *
 *  Created on: May 28, 2021
 *      Author: ACER
 */

#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include <stdint.h>
#include "mqtt.h"

uint8_t ping_json(char *buffer, uint8_t id_gateway);

uint8_t  head_provision(char *buffer,uint8_t device_id, char *device_name, char *device_type);
uint8_t  tail_provision(char *buffer,uint16_t channel_id, char *channel_name, char *channel_type, uint8_t function);

uint8_t timestamp_telemetry(char *buffer, uint8_t time[6]);
uint8_t head_telemetry(char *buffer, uint8_t device_id);
uint8_t tail_telemetry(char *buffer, uint16_t device_channel, char *value);

uint8_t command_read_json(char *buffer,uint8_t device_id, uint16_t device_channel,char *value);
uint8_t command_write_json(char *buffer,uint8_t device_id, uint16_t device_channel);

uint8_t topic(char *buffer, char *apikey, char *topic, char *flow_type);

#endif /* TELEMETRY_H_ */
