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

static struct tcp_socket socket;

static uint16_t packets_sent = 0;
static uint16_t packets_received = 0;

static data_struct *app_data;

#define INPUTBUFSIZE 400
static uint8_t inputbuf[INPUTBUFSIZE];

#define OUTPUTBUFSIZE 400
static uint8_t outputbuf[OUTPUTBUFSIZE];

static uint16_t seq_id_client = 0;
static uint16_t seq_id_server = 0;

static uint16_t bytes_received = 0;

/*---------------------------------------------------------------------------*/
PROCESS(tcp_server_process, "TCP server process");
AUTOSTART_PROCESSES(&tcp_server_process, &laneflood_process);
/*---------------------------------------------------------------------------*/
static void
tcpip_handler()
{
	if (seq_id_client < app_data->seq_id_client) {
		packets_received++;
		seq_id_client = app_data->seq_id_client;
		printf("cg%uch%ucl%u\n", packets_sent, packets_received, packetqueue_len(&tx_queue));

		app_data = (data_struct *) &outputbuf[0];
		sprintf(&app_data->data[0], "sssssssssssssssssssssssssssssssssssssssssssss");

		app_data->seq_id_server = ++seq_id_server;
		app_data->seq_id_client = seq_id_client;
		printf("cc(%u.%u)\n", app_data->seq_id_client, app_data->seq_id_server);
		packets_sent++;
		tcp_socket_send(&socket, &outputbuf[socket.output_data_len], (sizeof(app_data->seq_id_client) * 2)  + strlen(app_data->data));
	} else {
		printf("cn\n");
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
static int
input(struct tcp_socket *s, void *ptr,
		const uint8_t *inputptr, int inputdatalen)
{
	if (inputdatalen > 0) {
		if (socket.output_data_len == 0) {
			app_data = (data_struct *) &inputbuf[0];
			printf("cd(%u.%u)", app_data->seq_id_client, app_data->seq_id_server);
			tcpip_handler();
			if (!bytes_received) {
				bytes_received = inputdatalen;
			}
			return (bytes_received - inputdatalen);
		}
		return bytes_received;
	}
	if (!bytes_received) {
		return 0; /* all data consumed */
	}
	return bytes_received;
}
/*---------------------------------------------------------------------------*/
static void
event(struct tcp_socket *s, void *ptr,
		tcp_socket_event_t ev)
{
	if (ev == TCP_SOCKET_DATA_SENT) {
		if (s->output_data_len == 0) {
			if(seq_id_server % 4 != 0) {
				app_data = (data_struct *) &outputbuf[0];
				sprintf(&app_data->data[0], "sssssssssssssssssssssssssssssssssssssssssssss");

				app_data->seq_id_server = ++seq_id_server;
				app_data->seq_id_client = seq_id_client;
				printf("cc(%u.%u)\n", app_data->seq_id_client, app_data->seq_id_server);
				packets_sent++;
				tcp_socket_send(&socket, &outputbuf[socket.output_data_len], (sizeof(app_data->seq_id_client) * 2)  + strlen(app_data->data));
			}
		}
	}
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(tcp_server_process, ev, data)
{
	PROCESS_BEGIN();

	print_local_addresses();

	tcp_socket_register(&socket, NULL,
			inputbuf, sizeof(inputbuf),
			outputbuf, sizeof(outputbuf),
			input, event);
	tcp_socket_listen(&socket, 3000);

	while(1) {
		PROCESS_YIELD();
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
