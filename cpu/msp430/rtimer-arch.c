/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 * This file is part of the Contiki operating system.
 *
 * $Id: rtimer-arch.c,v 1.14 2010/02/18 22:15:54 adamdunkels Exp $
 */

/**
 * \file
 *         MSP430-specific rtimer code
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Martina Brachmann <martina.brachmann@tu-dresden.de>
 */

#include <legacymsp430.h>

#include "sys/energest.h"
#include "sys/rtimer.h"
#include "sys/process.h"

#include "dev/watchdog.h"
#include "isr_compat.h"
#include "dev/leds.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static rtimer32_clock_t currently_scheduled;
extern volatile rtimer_clock_t timer_overflows;

/*---------------------------------------------------------------------------*/
interrupt(TIMERA0_VECTOR) timera0 (void) {
	ENERGEST_ON(ENERGEST_TYPE_IRQ);

	watchdog_start();

	if(currently_scheduled == 0) { /* Timer expired */
		rtimer32_run_next();
	} else {
		rtimer32_arch_schedule_current();
	}

	if(process_nevents() > 0) {
		LPM4_EXIT;
	}

	watchdog_stop();

	ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_init(void) {
	dint();

	/* CCR0 interrupt enabled, interrupt occurs when timer equals CCR0. */
	TACCTL0 = CCIE;

	/* Enable interrupts. */
	eint();
}
/*---------------------------------------------------------------------------*/
void
rtimer32_arch_schedule_current() {
	rtimer32_clock_t now = RTIMER32_NOW();

	if(currently_scheduled == 0) {
		return;
	}

	if(currently_scheduled < now) {
		rtimer32_run_next();
		return;
	}

	if((currently_scheduled - now) & 0xffff0000) { /* Cannot be scheduled yet */
		TACCR0 = (uint16_t) (now + RTIMER_ARCH_SECOND);
	} else { /* Can be scheduled now */
		TACCR0 = (uint16_t) (currently_scheduled & 0xffff);
		currently_scheduled = 0;
	}
}
/*---------------------------------------------------------------------------*/
void
rtimer32_arch_schedule(rtimer32_clock_t t) {
	PRINTF("rtimer32_arch_schedule time %lu\n", t);
	{
		currently_scheduled = t;
		rtimer32_arch_schedule_current();
	}
}
/*---------------------------------------------------------------------------*/
volatile rtimer_clock_t timer_overflows;
/*---------------------------------------------------------------------------*/
rtimer_clock_t
rtimer32_arch_get_TAR_overflow(void) {
	rtimer32_arch_handle_TAR_overflow();
	return timer_overflows;
}
/*---------------------------------------------------------------------------*/
inline rtimer_clock_t
rtimer32_arch_handle_TAR_overflow(void) {
	while (TAR == 65535);
	// Two seconds are over TAR has wrapped around (the TAIFG flag is set)
	// 2^16 bit / 32786 Hz = 2 seconds
	if ((TACTL & TAIFG)) {
		++timer_overflows;
		//clear the interrupt pending flag
		TACTL &= ~TAIFG;
	}
	return TAR;
}
/*---------------------------------------------------------------------------*/
rtimer32_clock_t
rtimer32_arch_now(void) {
	// Check if we have a timer overflow of TAR, count it
	// and return the current TAR value
	rtimer_clock_t t1 = rtimer32_arch_handle_TAR_overflow();
	// Shift the counted overflow 16 bit to the left and add TAR to the lower 16 bit to get a virtual 32 bit timer
	return (((rtimer32_clock_t) timer_overflows) << 16) | t1;
}
/*---------------------------------------------------------------------------*/
