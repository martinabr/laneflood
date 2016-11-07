/*
* Copyright (c) 2016, TU Dresden.
* All rights reserved.
*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Author: Martina Brachmann <martina.brachmann@tu-dresden.de>
 *
 */

#include "contiki.h"
#include "contiki-net.h"

#include <string.h>
#include <stdio.h>

#include "udp-application.h"

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#include "../../core/net/mac/laneflood/laneflood.h"
#include "../../core/net/mac/laneflood/laneflood_interface.h"

static struct uip_udp_conn *server_conn;

static uint16_t packets_sent = 0;
static uint16_t packets_received = 0;

static data_struct *app_data;
static uint32_t buf[MAX_PAYLOAD_LEN/4];

static uint16_t seq_id_client = 0;
static uint16_t seq_id_server = 0;

/*---------------------------------------------------------------------------*/
PROCESS(udp_server_process, "UDP server process");
AUTOSTART_PROCESSES(&udp_server_process, &laneflood_process);
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
	if(uip_newdata()) {
		((char *)uip_appdata)[uip_datalen()] = 0;
		app_data = (data_struct *) &uip_appdata[0];
		printf("cd(%u.%u)", app_data->seq_id_client, app_data->seq_id_server);
		if (seq_id_client < app_data->seq_id_client) {
			packets_received++;
			seq_id_client = app_data->seq_id_client;
			printf("cg%uch%ucl%u\n", packets_sent, packets_received, packetqueue_len(&tx_queue));

			memset(&buf[0], 0, sizeof(buf));
			app_data = (data_struct *) &buf[0];
			sprintf(&app_data->data[0], "ssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss");

			uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
			app_data->seq_id_server = ++seq_id_server;
			app_data->seq_id_client = seq_id_client;
			printf("cc(%u.%u)\n", app_data->seq_id_client, app_data->seq_id_server);
			packets_sent++;
//			printf("bla %d \n", (sizeof(app_data->seq_id_client) * 2) + strlen(app_data->data));
//			printf("blabla %d \n", sizeof(app_data));
			uip_udp_packet_send(server_conn, buf, (sizeof(app_data->seq_id_client) * 2)  + strlen(app_data->data));

			app_data->seq_id_server = ++seq_id_server;
			app_data->seq_id_client = seq_id_client;
			printf("cc(%u.%u)\n", app_data->seq_id_client, app_data->seq_id_server);
			packets_sent++;
			uip_udp_packet_send(server_conn, buf, (sizeof(app_data->seq_id_client) * 2)  + strlen(app_data->data));

			app_data->seq_id_server = ++seq_id_server;
			app_data->seq_id_client = seq_id_client;
			printf("cc(%u.%u)\n", app_data->seq_id_client, app_data->seq_id_server);
			packets_sent++;
			uip_udp_packet_send(server_conn, buf, (sizeof(app_data->seq_id_client) * 2)  + strlen(app_data->data));

			app_data->seq_id_server = ++seq_id_server;
			app_data->seq_id_client = seq_id_client;
			printf("cc(%u.%u)\n", app_data->seq_id_client, app_data->seq_id_server);
			uip_udp_packet_send(server_conn, buf, (sizeof(app_data->seq_id_client) * 2)  + strlen(app_data->data));
			packets_sent++;

			app_data->seq_id_server = ++seq_id_server;
			app_data->seq_id_client = seq_id_client;
			printf("cc(%u.%u)\n", app_data->seq_id_client, app_data->seq_id_server);
			uip_udp_packet_send(server_conn, buf, (sizeof(app_data->seq_id_client) * 2)  + strlen(app_data->data));
			packets_sent++;

			printf("cl%d\n", packetqueue_len(&tx_queue));

			/* Restore server connection to allow data from any node */
			memset(&server_conn->ripaddr, 0, sizeof(server_conn->ripaddr));
		} else {
			printf("cn\n");
		}
	}
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
	int i;
	uint8_t state;

	PRINTF("Server IPv6 addresses: ");
	for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
		state = uip_ds6_if.addr_list[i].state;
		if(uip_ds6_if.addr_list[i].isused &&
				(state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
			PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
			PRINTF("\n");
		}
	}
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
	PROCESS_BEGIN();
	memset(&buf[0], 0, sizeof(buf));

	print_local_addresses();

	server_conn = udp_new(NULL, UIP_HTONS(3001), NULL);
	udp_bind(server_conn, UIP_HTONS(3000));

	while(1) {
		PROCESS_YIELD();
		if(ev == tcpip_event) {
			tcpip_handler();
		}
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
