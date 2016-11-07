/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 * $Id: cc2420.h,v 1.9 2010/02/23 18:24:49 adamdunkels Exp $
 */

/**
 * \file
 *         CC2420 driver header file
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#ifndef __CC2420_H__
#define __CC2420_H__

#include "contiki.h"
#include "cc2420_const.h"

int cc2420_init(void);
void cc2420_set_channel(int channel);

/*---------------------------------------------------------------------------*/
static inline uint8_t radio_status(void) {
	uint8_t status;
	FASTSPI_UPD_STATUS(status);
	return status;
}
static inline void radio_flush_rx(void) {
	uint8_t dummy;
	FASTSPI_READ_FIFO_BYTE(dummy);
	FASTSPI_STROBE(CC2420_SFLUSHRX);
	FASTSPI_STROBE(CC2420_SFLUSHRX);
}
/*---------------------------------------------------------------------------*/

#endif /* __CC2420_H__ */
