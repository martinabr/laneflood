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

#define SEND_INTERVAL  (11 * CLOCK_SECOND)

static uint8_t buf[MAX_PAYLOAD_LEN];


static data_struct *app_data;

static uint16_t seq_id_client = 0;
static uint16_t seq_id_server = 0;
static uint16_t seq_id_server_expected = 0;

static uint16_t packets_sent = 0;
static uint16_t packets_received = 0;

#define MAX_DESTINATIONS 3
static uint16_t seq_id_server_expected_list[MAX_DESTINATIONS] = {0, 0, 0};
static uint16_t seq_id_server_list[MAX_DESTINATIONS] = {0, 0, 0};
static uint8_t destination_index = 0;

static struct uip_udp_conn *client_conn;
/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_client_process, &laneflood_process);
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void) {
    seq_id_server_expected++;
    if (uip_newdata()) {
        ((char *) uip_appdata)[uip_datalen()] = 0;
        app_data = (data_struct *) & uip_appdata[0];
        printf("cb(%u.%u)cf", app_data->seq_id_client, app_data->seq_id_server);
        PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
        if (seq_id_server < app_data->seq_id_server) {
            packets_received++;
            seq_id_server = app_data->seq_id_server;
            if (seq_id_server_expected < app_data->seq_id_server) {
                seq_id_server_expected = app_data->seq_id_server;
            }
            printf("ci%ucj%uck%u\n", packets_sent, packets_received, packetqueue_len(&tx_queue));
        } else {
            printf("cm\n");
            seq_id_server_expected--;
        }
    }
}
/*---------------------------------------------------------------------------*/
static void
timeout_handler(void) {
    memset(&buf[0], 0, sizeof (buf));
    app_data = (data_struct *) & buf[0];

    app_data->seq_id_client = ++seq_id_client;
    app_data->seq_id_server = seq_id_server;
    
    sprintf(&app_data->data[0], "ccccccccccccccccccccccccc");

    printf("ce");
    PRINT6ADDR(&client_conn->ripaddr);
    printf("ca(%u.%u)", app_data->seq_id_client, app_data->seq_id_server);
    packets_sent++;
    uip_udp_packet_send(client_conn, buf, (sizeof (app_data->seq_id_client) * 2) + strlen(app_data->data));
    printf("ck%u\n", packetqueue_len(&tx_queue));
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void) {
    int i;
    uint8_t state;

    PRINTF("Client IPv6 addresses: ");
    for (i = 0; i < UIP_DS6_ADDR_NB; i++) {
        state = uip_ds6_if.addr_list[i].state;
        if (uip_ds6_if.addr_list[i].isused &&
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
PROCESS_THREAD(udp_client_process, ev, data) {
    static struct etimer et;
    static uip_ipaddr_t ipaddr;

    PROCESS_BEGIN();

    print_local_addresses();

    etimer_set(&et, CLOCK_SECOND * 60);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    uip_ip6addr(&ipaddr,
            server_addr[destination_index][0],
            server_addr[destination_index][1],
            server_addr[destination_index][2],
            server_addr[destination_index][3],
            server_addr[destination_index][4],
            server_addr[destination_index][5],
            server_addr[destination_index][6],
            server_addr[destination_index][7]);
    /* new connection with remote host */
    client_conn = udp_new(&ipaddr, UIP_HTONS(3000), NULL);
    udp_bind(client_conn, UIP_HTONS(3001));
    printf("Connect to ");
    PRINT6ADDR(&client_conn->ripaddr);
    printf(" %u/%u\n",
            UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));

    etimer_set(&et, (random_rand() % SEND_INTERVAL) + CLOCK_SECOND);
    while (1) {
        PROCESS_YIELD();
        if (etimer_expired(&et)) {
            if (((((int8_t) seq_id_client + 1) & 0x7e) == 0x7e) ||
                    ((((int8_t) seq_id_client + 1) & 0x7f) == 0x7f)) {
                seq_id_client++;
            } else {
                uint8_t old_destination_index = destination_index;
                destination_index = (((0xffff & (seq_id_client + 1)) / 128) % MAX_DESTINATIONS);
                if (old_destination_index != destination_index) {
                    uip_ip6addr(&ipaddr,
                            server_addr[destination_index][0],
                            server_addr[destination_index][1],
                            server_addr[destination_index][2],
                            server_addr[destination_index][3],
                            server_addr[destination_index][4],
                            server_addr[destination_index][5],
                            server_addr[destination_index][6],
                            server_addr[destination_index][7]);
                    /* new connection with remote host */
                    client_conn = udp_new(&ipaddr, UIP_HTONS(3000), NULL);
                    udp_bind(client_conn, UIP_HTONS(3001));
                    printf("Connected to ");
                    PRINT6ADDR(&client_conn->ripaddr);
                    printf(" %u/%u\n",
                            UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));
                    // Store the current seq_id_server_expected and get the new one
                    seq_id_server_expected_list[old_destination_index] = seq_id_server_expected;
                    seq_id_server_expected = seq_id_server_expected_list[destination_index];
                    seq_id_server_list[old_destination_index] = seq_id_server;
                    seq_id_server = seq_id_server_list[destination_index];
                }
                timeout_handler();
            }
            etimer_set(&et, (random_rand() % SEND_INTERVAL) + CLOCK_SECOND);
        } else if (ev == tcpip_event) {
            tcpip_handler();
        }
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/