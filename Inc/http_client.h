/*
 * http_client.h
 *
 *  Created on: Mar 30, 2021
 *      Author: ACER
 */

#ifndef HTTP_CLIENT_H_
#define HTTP_CLIENT_H_
#include "err.h"
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/apps/fs.h"

void http_client_init(void);
void http_client_send_packet(struct netconn *conn);

#endif /* HTTP_CLIENT_H_ */
