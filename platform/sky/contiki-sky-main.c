/*
 * Copyright (c) 2006, Swedish Institute of Computer Science
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
 * @(#)$Id: contiki-sky-main.c,v 1.78 2010/03/19 14:08:15 joxe Exp $
 */

#include <legacymsp430.h>
#include <stdio.h>
#include <string.h>
#include "../../core/net/mac/laneflood/laneflood.h"
#include "contiki.h"

#include "dev/cc2420.h"
#include "dev/leds.h"
#include "dev/uart1.h"
#include "dev/watchdog.h"
#include "dev/xmem.h"

#include "dev/ds2411/ds2411.h"

#include "sys/node-id.h"
#include "sys/autostart.h"
#include "lib/random.h"

#include "net/ip/uip.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"

#if NETSTACK_CONF_WITH_IPV6
#include "net/ipv6/uip-ds6.h"
#endif /* NETSTACK_CONF_WITH_IPV6 */

#if TINYOS_SERIAL_FRAMES
// they will be changed before the upload
unsigned short TOS_NODE_ID = 0x1122;
unsigned short TOS_LOCAL_ADDRESS = 0x3344;
#endif

#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else /* DEBUG */
#define PRINTF(...)
#endif /* DEBUG */

/*---------------------------------------------------------------------------*/
void uip_log(char *msg) { puts(msg); }

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static void
set_rime_addr(void)
{
	linkaddr_t addr;
	int i;

	memset(&addr, 0, sizeof(linkaddr_t));
#if NETSTACK_CONF_WITH_IPV6
	//printf("addr.u8 %d \n", sizeof(addr.u8));
	//memcpy(addr.u8, ds2411_id, sizeof(addr.u8));
	memcpy(addr.u8, ds2411_id, sizeof(addr.u8)-2);
	addr.u8[sizeof(addr.u8)-3] = node_id;
	addr.u8[sizeof(addr.u8)-2] = 0;
#else
	if(node_id == 0) {
		for(i = 0; i < sizeof(linkaddr_t); ++i) {
			addr.u8[i] = ds2411_id[7 - i];
		}
	} else {
		addr.u8[0] = node_id & 0xff;
		addr.u8[1] = node_id >> 8;
	}
#endif
	linkaddr_set_node_addr(&addr);
	  PRINTF("Rime started with address ");
	  for(i = 0; i < sizeof(addr.u8) - 1; i++) {
	    PRINTF("%d.", addr.u8[i]);
	  }
	  PRINTF("%d\n", addr.u8[i]);
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
static void
print_processes(struct process * const processes[])
{
	/*  const struct process * const * p = processes;*/
	printf("Starting");
	while(*processes != NULL) {
		printf(" '%s'", (*processes)->name);
		processes++;
	}
	putchar('\n');
}
/*--------------------------------------------------------------------------*/
int
main(int argc, char **argv)
{
	/*
	 * Initalize hardware.
	 */
	msp430_cpu_init();
	clock_init();
	leds_init();
	leds_on(LEDS_RED);

	uart1_init(BAUD2UBR(115200)); /* Must come before first printf */

	leds_on(LEDS_GREEN);

	ds2411_init();
	/* XXX hack: Fix it so that the 802.15.4 MAC address is compatible
	   with an Ethernet MAC address - byte 0 (byte 2 in the DS ID)
	   cannot be odd. */
	ds2411_id[2] &= 0xfe;

	leds_on(LEDS_BLUE);
	xmem_init();
	random_init(ds2411_id[0] + node_id);

	leds_off(LEDS_RED);
	rtimer_init();
	/*
	 * Hardware initialization done!
	 */

	/* Initialize energest first (but after rtimer) */
	energest_init();
	ENERGEST_ON(ENERGEST_TYPE_CPU);

#if BURN_NODE_ID
	node_id = LOCAL_NODE_ID;
	node_id_burn(node_id);
#endif /* BURN_NODE_ID */

#if TINYOS_SERIAL_FRAMES
	node_id = TOS_NODE_ID;
#else
	/* Restore node id if such has been stored in external mem */
	node_id_restore();
#endif

	leds_off(LEDS_BLUE);
	/*
	 * Initialize Contiki and our processes.
	 */
	process_init();
	process_start(&etimer_process, NULL);

	set_rime_addr();

	cc2420_init();
	cc2420_set_channel(RF_CHANNEL);

	{
		uint8_t longaddr[8];
		memset(longaddr, 0, sizeof(longaddr));
		linkaddr_copy((linkaddr_t *)&longaddr, &linkaddr_node_addr);
	}

	printf(CONTIKI_VERSION_STRING " started. ");
	if(node_id > 0) {
		printf("Node id is set to %u.\n", node_id);
	} else {
		printf("Node id is not set.\n");
	}

#if NETSTACK_CONF_WITH_IPV6
//	memcpy(&uip_lladdr.addr, ds2411_id, sizeof(uip_lladdr.addr));
//	printf("uip_ll %d \n", sizeof(uip_lladdr.addr));
	memcpy(&uip_lladdr.addr, ds2411_id, sizeof(uip_lladdr.addr)-2);
	uip_lladdr.addr[sizeof(uip_lladdr.addr)-2] = 0;
	uip_lladdr.addr[sizeof(uip_lladdr.addr)-1] = node_id;

	queuebuf_init();
	NETSTACK_LLSEC.init();
	NETSTACK_NETWORK.init();
	
	process_start(&tcpip_process, NULL);

	if(!UIP_CONF_IPV6_RPL) {
		uip_ipaddr_t ipaddr;
		uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
		uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
		uip_ds6_addr_add(&ipaddr, 0, ADDR_TENTATIVE);
	}
#else /* NETSTACK_CONF_WITH_IPV6 */

	queuebuf_init();
	NETSTACK_LLSEC.init();
	NETSTACK_NETWORK.init();
	
	process_start(&tcpip_process, NULL);

#endif /* NETSTACK_CONF_WITH_IPV6 */


	leds_off(LEDS_GREEN);

	watchdog_start();

	print_processes(autostart_processes);
	autostart_start(autostart_processes);

	/*
	 * This is the scheduler loop.
	 */
	while(1) {
		int r;
		do {
			/* Reset watchdog. */
			watchdog_periodic();
			r = process_run();
		} while(r > 0);

		/*
		 * Idle processing.
		 */
		int s = splhigh();		/* Disable interrupts. */
		/* uart1_active is for avoiding LPM3 when still sending or receiving */
		if(process_nevents() != 0 || uart1_active()) {
			splx(s);			/* Re-enable interrupts. */
		} else {
			static unsigned long irq_energest = 0;

			/* Re-enable interrupts and go to sleep atomically. */
			ENERGEST_OFF(ENERGEST_TYPE_CPU);
			ENERGEST_ON(ENERGEST_TYPE_LPM);
			/* We only want to measure the processing done in IRQs when we
	 are asleep, so we discard the processing time done when we
	 were awake. */
			energest_type_set(ENERGEST_TYPE_IRQ, irq_energest);
			watchdog_stop();
			_BIS_SR(GIE | SCG0 | SCG1 | CPUOFF); /* LPM3 sleep. This
					      statement will block
					      until the CPU is
					      woken up by an
					      interrupt that sets
					      the wake up flag. */
			/* We get the current processing time for interrupts that was
	 done during the LPM and store it for next time around.  */
			dint();
			irq_energest = energest_type_time(ENERGEST_TYPE_IRQ);
			eint();
			watchdog_start();
			ENERGEST_OFF(ENERGEST_TYPE_LPM);
			ENERGEST_ON(ENERGEST_TYPE_CPU);
		}
	}

	return 0;
}
