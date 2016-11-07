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

#include "laneflood_interface.h"
#include <stdio.h>

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"


LIST(tx_queue_list);
MEMB(tx_queue_memb, struct packetqueue_item, MAX_TX_QUEUE);

struct packetqueue tx_queue = { &tx_queue_list, 
                                &tx_queue_memb };

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static void
llsec_init() {
}
/*---------------------------------------------------------------------------*/
static void
llsec_send(mac_callback_t send, void *ptr) {
    // Copy everthing to packetbuf_prt (payload + UDP/TCP header + IP header + LLSEC header
    memcpy(packetbuf_dataptr(), uip_buf, uip_len);
    packetbuf_set_datalen(uip_len);
    uip_clear_buf();

    // Copy packet in packetbuf to the packetqueue
    int ret = packetqueue_enqueue_packetbuf(&tx_queue, 0, NULL);
    if (!ret) {
        printf("[ERROR] Could not queue packet. Returned %d \n", ret);
        return;
    }
    //printf("enqueued packet \n");
    packetbuf_clear();
}
/*---------------------------------------------------------------------------*/
static void
llsec_input(void) {
    NETSTACK_NETWORK.input();
}
/*---------------------------------------------------------------------------*/
const struct llsec_driver laneflood_llsec_driver = {
    "laneflood_llsec_driver",
    llsec_init,
    llsec_send,
    llsec_input
};
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static uint8_t
output(const uip_lladdr_t *localdest) {

    packetbuf_clear();

    linkaddr_t dest;
    linkaddr_copy(&dest, (const linkaddr_t *) localdest + 1); // We want the last 8 bit, the destination node id is there
    packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, &dest);

    // call send
    NETSTACK_LLSEC.send(NULL, NULL);

    return 1;
}
/*---------------------------------------------------------------------------*/
void
network_init(void) {
    packetqueue_init(&tx_queue);
    // set the output function for tcpip
    tcpip_set_outputfunc(output);
}
/*---------------------------------------------------------------------------*/
static void
network_input(void) {
    memcpy(uip_buf, packetbuf_dataptr(), packetbuf_datalen());
    uip_len = packetbuf_datalen();

    tcpip_input();
}
/*---------------------------------------------------------------------------*/
const struct network_driver laneflood_network_driver = {
    "laneflood_network_driver",
    network_init,
    network_input,
};
/*---------------------------------------------------------------------------*/