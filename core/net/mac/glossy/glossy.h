/*
 * Copyright (c) 2011, ETH Zurich.
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
 * Author: Federico Ferrari <ferrari@tik.ee.ethz.ch>
 *         Martina Brachmann <martina.brachmann@tu-dresden.de>
 *
 */

/**
 * \file
 *         Glossy core, header file.
 */

#ifndef GLOSSY_H_
#define GLOSSY_H_

#include "contiki.h"
#include "dev/watchdog.h"
#include "dev/cc2420_const.h"
#include "dev/leds.h"
#include "dev/spi.h"
#include <stdio.h>
#include <legacymsp430.h>
#include <msp430f1611.h>
#include <stdlib.h>

/**
 * If not zero, nodes print additional debug information (disabled by default).
 */
#ifndef GLOSSY_DEBUG
#define GLOSSY_DEBUG 0
#endif
#ifndef GLOSSY_DEBUG_PINS
#define GLOSSY_DEBUG_PINS 0
#endif
#ifndef GLOSSY_DEBUG_STATUS
#define GLOSSY_DEBUG_STATUS 0
#endif
/**
 * Size of the window used to average estimations of slot lengths.
 */
#define GLOSSY_SYNC_WINDOW            64
/**
 * Initiator timeout, in number of Glossy slots.
 * When the timeout expires, if the initiator has not received any packet
 * after its first transmission it transmits again.
 */
#define GLOSSY_INITIATOR_TIMEOUT      3

/**
 * Ratio between the frequencies of the DCO and the low-frequency clocks
 */
#if COOJA
#define CLOCK_PHI                     (4194304uL / RTIMER_SECOND)
#else
#define CLOCK_PHI                     (F_CPU / RTIMER_SECOND)
#endif /* COOJA */

#define GLOSSY_RELAY_CNT_LEN          sizeof(uint8_t)
#define GLOSSY_IS_ON()                (get_state() != GLOSSY_STATE_OFF)
#define FOOTER_LEN                    2
#define FOOTER1_CRC_OK                0x80
#define FOOTER1_CORRELATION           0x7f

/**
 * The Glossy header fields
 */
typedef struct {
	uint8_t	fs_len_field;		/* The length of the packet (needs to be set for the radio) */
	uint8_t fs_relay_cnt_field;	/* The relay counter field */
	uint8_t fs_payload[];		/* Pointer to the next header or data struct */
} __attribute__((packed)) fs_header_glossy_struct;

#define GLOSSY_RSSI_FIELD             packet[packet_len_tmp - 1]
#define GLOSSY_CRC_FIELD              packet[packet_len_tmp]

enum {
	GLOSSY_INITIATOR = 1, GLOSSY_RECEIVER = 0
};

/**
 * List of possible Glossy states.
 */
enum glossy_state {
	GLOSSY_STATE_OFF,          /**< Glossy is not executing */
	GLOSSY_STATE_WAITING,      /**< Glossy is waiting for a packet being flooded */
	GLOSSY_STATE_RECEIVING,    /**< Glossy is receiving a packet */
	GLOSSY_STATE_RECEIVED,     /**< Glossy has just finished receiving a packet */
	GLOSSY_STATE_TRANSMITTING, /**< Glossy is transmitting a packet */
	GLOSSY_STATE_TRANSMITTED,  /**< Glossy has just finished transmitting a packet */
	GLOSSY_STATE_ABORTED       /**< Glossy has just aborted a packet reception */
};
#if GLOSSY_DEBUG
uint16_t high_T_irq, rx_timeout, bad_length, bad_crc;
#endif /* GLOSSY_DEBUG */
#if GLOSSY_DEBUG_STATUS
uint8_t debug_status_index;
enum debug_status_list {
	DEBUG_WAITING,
	DEBUG_BEGIN_RX_START,
	DEBUG_BEGIN_RX_STOP,
	DEBUG_BEGIN_RX_TIMEOUT_FIRST_BYTE,
	DEBUG_BEGIN_RX_BAD_LENGHT,
	DEBUG_BEGIN_RX_TIMEOUT_REST_BYTE,
	DEBUG_END_RX_START,
	DEBUG_END_RX_STOP,
	DEBUG_END_RX_BAD_CRX,
	DEBUG_END_RX_TX_MAX,
	DEBUG_BEGIN_TX_START,
	DEBUG_BEGIN_TX_STOP,
	DEBUG_END_TX_START,
	DEBUG_END_TX_STOP,
	DEBUG_END_TX_TX_MAX,
	DEBUG_T_IRQ_HIGH,
	DEBUG_INITIATOR_TIMEOUT,
	DEBUG_RECEIVER_TIMEOUT,
	//DEBUG_STXON,
	//DEBUG_SRXON,
	DEBUG_SRF_OFF,
	DEBUG_UNKNOWN
};
uint8_t debug_status[250];
#endif /* GLOSSY_DEBUG_STATUS */

PROCESS_NAME(glossy_process);

/* ----------------------- Application interface -------------------- */
/**
 * \defgroup glossy_interface Glossy API
 * @{
 * \file   glossy.h
 * \file   glossy.c
 */

/**
 * \defgroup glossy_main Interface related to flooding
 * @{
 */

/**
 * \brief            		Start Glossy and stall all other application tasks.
 *
 * \param data_      		A pointer to the flooding data.
 *
 *                   		At the initiator, Glossy reads from the given memory
 *                   		location data provided by the application.
 *
 *                   		At a receiver, Glossy writes to the given memory
 *                   		location data for the application.
 * \param data_len_  		Length of the flooding data, in bytes.
 * \param initiator_ 		Not zero if the node is the initiator,
 *                  		zero if it is a receiver.
 * \param tx_max_    		Maximum number of transmissions (N).
 * \param t_stop_    		Time instant at which Glossy must stop, in case it is
 *                   		still running.
 * \param cb_        		Callback function, called when Glossy terminates its
 *                   		execution.
 * \param rtimer_    		First argument of the callback function.
 * \param ptr_       		Second argument of the callback function.
 */
void glossy_start(uint8_t *data_, uint8_t *data_len_, uint8_t initiator_,
		uint8_t sync_, uint8_t tx_max_, rtimer32_clock_t t_stop_, rtimer_callback_t cb_,
		struct rtimer *rtimer_, void *ptr_);

/**
 * \brief            Stop Glossy and resume all other application tasks.
 * \returns          Number of times the packet has been received during
 *                   last Glossy phase.
 *                   If it is zero, the packet was not successfully received.
 * \sa               get_rx_cnt
 */
uint8_t glossy_stop(void);

/**
 * \brief            Get the last received counter.
 * \returns          Number of times the packet has been received during
 *                   last Glossy phase.
 *                   If it is zero, the packet was not successfully received.
 */
uint8_t get_rx_cnt(void);

/**
 * \brief            Get the current Glossy state.
 * \return           Current Glossy state, one of the possible values
 *                   of \link glossy_state \endlink.
 */
uint8_t get_state(void);
/**
 * \brief            Get low-frequency time of first packet reception
 *                   during the last Glossy phase.
 * \returns          Low-frequency time of first packet reception
 *                   during the last Glossy phase.
 */
rtimer_clock_t get_t_first_rx_l(void);

/** @} */

/**
 * \defgroup glossy_sync Interface related to time synchronization
 * @{
 */

/**
 * \brief            Get the last relay counter.
 * \returns          Value of the relay counter embedded in the first packet
 *                   received during the last Glossy phase.
 */
uint8_t get_relay_cnt(void);

/**
 * \brief            Get the local estimation of T_slot, in DCO clock ticks.
 * \returns          Local estimation of T_slot.
 */
rtimer_clock_t get_T_slot_h(void);

/**
 * \brief            Get low-frequency synchronization reference time.
 * \returns          Low-frequency reference time
 *                   (i.e., time at which the initiator started the flood).
 */
rtimer_clock_t get_t_ref_l(void);

/**
 * \brief            Provide information about current synchronization status.
 * \returns          Not zero if the synchronization reference time was
 *                   updated during the last Glossy phase, zero otherwise.
 */
uint8_t is_t_ref_l_updated(void);

/**
 * \brief            Set low-frequency synchronization reference time.
 * \param t          Updated reference time.
 *                   Useful to manually update the reference time if a
 *                   packet has not been received.
 */
void set_t_ref_l(rtimer_clock_t t);

/**
 * \brief            Set the current synchronization status.
 * \param updated    Not zero if a node has to be considered synchronized,
 *                   zero otherwise.
 */
void set_t_ref_l_updated(uint8_t updated);

// Used only for Debugging

uint8_t *get_packet(void);

uint8_t get_packet_len(void);

void glossy_reset(void);
/** @} */

/** @} */

/**
 * \defgroup glossy_internal Glossy internal functions
 * @{
 * \file   glossy.h
 * \file   glossy.c
 */

/* ------------------------------ Timeouts -------------------------- */
/**
 * \defgroup glossy_timeouts Timeouts
 * @{
 */

inline void glossy_schedule_rx_timeout(void);
inline void glossy_stop_rx_timeout(void);
inline void glossy_schedule_initiator_timeout(void);
inline void glossy_stop_initiator_timeout(void);

/** @} */

/* ----------------------- Interrupt functions ---------------------- */
/**
 * \defgroup glossy_interrupts Interrupt functions
 * @{
 */

inline void glossy_begin_rx(void);
inline void glossy_end_rx(void);
inline void glossy_begin_tx(void);
inline void glossy_end_tx(void);

/** @} */

/**
 * \defgroup glossy_capture Timer capture of clock ticks
 * @{
 */

/* -------------------------- Clock Capture ------------------------- */
/**
 * \brief Capture next low-frequency clock tick and DCO clock value at that instant.
 * \param t_cap_h variable for storing value of DCO clock value
 * \param t_cap_l variable for storing value of low-frequency clock value
 */
#define CAPTURE_NEXT_CLOCK_TICK(t_cap_h, t_cap_l) do {\
		/* Enable capture mode for timers B6 and A2 (ACLK) */\
		TBCCTL6 = CCIS0 | CM_POS | CAP | SCS; \
		TACCTL2 = CCIS0 | CM_POS | CAP | SCS; \
		/* Wait until both timers capture the next clock tick */\
		while (!((TBCCTL6 & CCIFG) && (TACCTL2 & CCIFG))); \
		/* Store the capture timer values */\
		t_cap_h = TBCCR6; \
		t_cap_l = TACCR2; \
		/* Disable capture mode */\
		TBCCTL6 = 0; \
		TACCTL2 = 0; \
} while (0)

/** @} */

/* -------------------------------- SFD ----------------------------- */

/**
 * \defgroup glossy_sfd Management of SFD interrupts
 * @{
 */

/**
 * \brief Capture instants of SFD events on timer B1
 * \param edge Edge used for capture.
 *
 */
#define SFD_CAP_INIT(edge) do {\
	P4SEL |= BV(SFD);\
	TBCCTL1 = edge | CAP | SCS;\
} while (0)

/**
 * \brief Enable generation of interrupts due to SFD events
 */
#define ENABLE_SFD_INT()		do { TBCCTL1 |= CCIE; } while (0)

/**
 * \brief Disable generation of interrupts due to SFD events
 */
#define DISABLE_SFD_INT()		do { TBCCTL1 &= ~CCIE; } while (0)

/**
 * \brief Clear interrupt flag due to SFD events
 */
#define CLEAR_SFD_INT()			do { TBCCTL1 &= ~CCIFG; } while (0)

/**
 * \brief Check if generation of interrupts due to SFD events is enabled
 */
#define IS_ENABLED_SFD_INT()    !!(TBCCTL1 & CCIE)

/** @} */

#define SET_PIN(a,b)          do { P##a##OUT |=  BV(b); } while (0)
#define UNSET_PIN(a,b)        do { P##a##OUT &= ~BV(b); } while (0)
#define TOGGLE_PIN(a,b)       do { P##a##OUT ^=  BV(b); } while (0)
#define INIT_PIN_IN(a,b)      do { P##a##SEL &= ~BV(b); P##a##DIR &= ~BV(b); } while (0)
#define INIT_PIN_OUT(a,b)     do { P##a##SEL &= ~BV(b); P##a##DIR |=  BV(b); } while (0)
#define PIN_IS_SET(a,b)       (    P##a##IN  &   BV(b))

// ADC2 (P6.2) -> LED3 (assume blue)
#define SET_PIN_ADC2         SET_PIN(6,2)
#define UNSET_PIN_ADC2       UNSET_PIN(6,2)
#define TOGGLE_PIN_ADC2      TOGGLE_PIN(6,2)
#define INIT_PIN_ADC2_IN     INIT_PIN_IN(6,2)
#define INIT_PIN_ADC2_OUT    INIT_PIN_OUT(6,2)
#define PIN_ADC2_IS_SET      PIN_IS_SET(6,2)

// ADC6 (P6.6) -> LED2 (assume green)
#define SET_PIN_ADC6         SET_PIN(6,6)
#define UNSET_PIN_ADC6       UNSET_PIN(6,6)
#define TOGGLE_PIN_ADC6      TOGGLE_PIN(6,6)
#define INIT_PIN_ADC6_IN     INIT_PIN_IN(6,6)
#define INIT_PIN_ADC6_OUT    INIT_PIN_OUT(6,6)
#define PIN_ADC6_IS_SET      PIN_IS_SET(6,6)

// ADC7 (P6.7) -> LED1 (assume red)
#define SET_PIN_ADC7         SET_PIN(6,7)
#define UNSET_PIN_ADC7       UNSET_PIN(6,7)
#define TOGGLE_PIN_ADC7      TOGGLE_PIN(6,7)
#define INIT_PIN_ADC7_IN     INIT_PIN_IN(6,7)
#define INIT_PIN_ADC7_OUT    INIT_PIN_OUT(6,7)
#define PIN_ADC7_IS_SET      PIN_IS_SET(6,7)

#endif /* GLOSSY_H_ */

/** @} */
