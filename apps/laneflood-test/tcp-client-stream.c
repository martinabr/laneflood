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
#include "destinations.h"

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#include "../../core/net/mac/laneflood/laneflood.h"
#include "../../core/net/mac/laneflood/laneflood_interface.h"

#include "lib/random.h"

#define SEND_INTERVAL		8 * CLOCK_SECOND

static data_struct *app_data;

static uint16_t seq_id_client = 0;
static uint16_t seq_id_server = 0;
static uint16_t seq_id_server_expected = 0;
static uint16_t packets_sent = 0;
static uint16_t packets_received = 0;

static struct tcp_socket socket;

#define INPUTBUFSIZE 400
static uint8_t inputbuf[INPUTBUFSIZE];

#define OUTPUTBUFSIZE 400
static uint8_t outputbuf[OUTPUTBUFSIZE];

static uint16_t bytes_received = 0;
/*---------------------------------------------------------------------------*/
PROCESS(tcp_client_process, "TCP client process");
AUTOSTART_PROCESSES(&tcp_client_process, &laneflood_process);
/*---------------------------------------------------------------------------*/
static void
tcpip_handler()
{

	seq_id_server_expected++;
	PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
	if (seq_id_server < app_data->seq_id_server) {
		packets_received++;
		seq_id_server = app_data->seq_id_server;
		if (seq_id_server_expected < app_data->seq_id_server) {
			seq_id_server_expected = app_data->seq_id_server;
		}
		printf("ci%ucj%uck%u\n", packets_sent, packets_received, packetqueue_len(&rx_queue));
	} else {
		printf("cm\n");
		seq_id_server_expected--;
	}
}
/*---------------------------------------------------------------------------*/
static int
input(struct tcp_socket *s, void *ptr,
		const uint8_t *inputptr, int inputdatalen)
{
	if (inputdatalen > 0) {
		app_data = (data_struct *) &inputbuf[0];
		printf("cb(%u.%u)cf", app_data->seq_id_client, app_data->seq_id_server);
		tcpip_handler(inputdatalen);
		if (!bytes_received) {
			bytes_received = inputdatalen;
		}
		return (bytes_received - inputdatalen);
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
}
/*---------------------------------------------------------------------------*/
static void
timeout_handler(void)
{
	memset(&outputbuf[0], 0, sizeof(outputbuf));
	app_data = (data_struct *) &outputbuf[0];

	app_data->seq_id_client = ++seq_id_client;
	app_data->seq_id_server = seq_id_server;
	sprintf(&app_data->data[0], "ccccccccccccccccccccccccccccccccccccccccccc");

	printf("ce");
	PRINT6ADDR(&(*(socket.c)).ripaddr);
	printf("ca(%u.%u)", app_data->seq_id_client, app_data->seq_id_server);
	packets_sent++;
	tcp_socket_send(&socket, outputbuf, (sizeof(app_data->seq_id_client) * 2)  + strlen(app_data->data));
	printf("ck%u\n", packetqueue_len(&rx_queue));
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
	int i;
	uint8_t state;

	PRINTF("Client IPv6 addresses: ");
	for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
		state = uip_ds6_if.addr_list[i].state;
		if(uip_ds6_if.addr_list[i].isused &&
				(state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
			PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
			PRINTF("\n");
		}
		/* hack to make address "final" */
		if (state == ADDR_TENTATIVE) {
			uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
		}
	}
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(tcp_client_process, ev, data)
{
	static struct etimer et;
	static uip_ipaddr_t ipaddr;

	PROCESS_BEGIN();

	print_local_addresses();

	uip_ip6addr(&ipaddr, server_addr1, server_addr2, server_addr3, server_addr4, server_addr5, server_addr6, server_addr7, server_addr8);

	etimer_set(&et, CLOCK_SECOND * 60);
	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

	tcp_socket_register(&socket, NULL,
	               inputbuf, sizeof(inputbuf),
	               outputbuf, sizeof(outputbuf),
	               input, event);

	/* new connection with remote host */
	int connect = tcp_socket_connect(&socket, &ipaddr, 3000);

	if (!connect) {
		printf("[ERROR] Cannot connect \n");
	}

	printf("Created a connection with the server ");
	PRINT6ADDR(&(*(socket.c)).ripaddr);
	printf(" local/remote port %u/%u\n",
			UIP_HTONS((*(socket.c)).lport), UIP_HTONS((*(socket.c)).rport));

	etimer_set(&et, random_rand() % (SEND_INTERVAL * 2) + CLOCK_SECOND);
	while(1) {
		PROCESS_YIELD();
		if(etimer_expired(&et)) {
			timeout_handler();
			etimer_set(&et, random_rand() % (SEND_INTERVAL * 2) + CLOCK_SECOND);
		}
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
