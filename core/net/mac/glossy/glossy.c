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
 *         Glossy core, source file.
 */

#include "glossy.h"
#include "cc2420.h"

#define CM_POS              CM_1
#define CM_NEG              CM_2
#define CM_BOTH             CM_3

static uint8_t initiator, sync, rx_cnt, tx_cnt, tx_max;
/* The declarations below ensure that the packet buffer is aligned on
   an even 32-bit boundary. On some platforms (most notably the
   msp430 or OpenRISC), having a potentially misaligned packet buffer may lead to
   problems when accessing words.
   From Contiki's packetbuf.c*/
static uint32_t packet_aligned[128 / 4];
static uint8_t *data;
static uint8_t *packet;
static uint8_t packet_len, packet_len_tmp;
static uint8_t *data_len;
static fs_header_glossy_struct * fs_header_glossy;
static uint8_t bytes_read, tx_relay_cnt_last, n_timeouts;
static volatile uint8_t state;
static rtimer_clock_t t_rx_start, t_rx_stop, t_tx_start, t_tx_stop, t_start;
static rtimer_clock_t t_rx_timeout;
static rtimer_clock_t T_irq;
static rtimer32_clock_t t_stop;
static rtimer_callback_t cb;
static struct rtimer *rtimer;
static void *ptr;
static unsigned short ie1, ie2, p1ie, p2ie, tbiv;

static rtimer_clock_t T_slot_h, T_rx_h, T_w_rt_h, T_tx_h, T_w_tr_h, t_ref_l, T_offset_h, t_first_rx_l;
#if GLOSSY_SYNC_WINDOW
static unsigned long T_slot_h_sum;
static uint8_t win_cnt;
#endif /* GLOSSY_SYNC_WINDOW */
static uint8_t relay_cnt, t_ref_l_updated;

/* --------------------------- Radio functions ---------------------- */
static inline void radio_flush_tx(void) {
    FASTSPI_STROBE(CC2420_SFLUSHTX);
}

//static inline uint8_t radio_status(void) {
//	uint8_t status;
//	FASTSPI_UPD_STATUS(status);
//	return status;
//}

static inline void radio_on(void) {
    FASTSPI_STROBE(CC2420_SRXON);
    while (!(radio_status() & (BV(CC2420_XOSC16M_STABLE))));
    ENERGEST_ON(ENERGEST_TYPE_LISTEN);
}

static inline void radio_off(void) {
#if ENERGEST_CONF_ON
    if (energest_current_mode[ENERGEST_TYPE_TRANSMIT]) {
        ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
    }
    if (energest_current_mode[ENERGEST_TYPE_LISTEN]) {
        ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
    }
#endif /* ENERGEST_CONF_ON */
    FASTSPI_STROBE(CC2420_SRFOFF);
}

//static inline void radio_flush_rx(void) {
//	uint8_t dummy;
//	FASTSPI_READ_FIFO_BYTE(dummy);
//	FASTSPI_STROBE(CC2420_SFLUSHRX);
//	FASTSPI_STROBE(CC2420_SFLUSHRX);
//}

static inline void radio_abort_rx(void) {
    state = GLOSSY_STATE_ABORTED;
    radio_flush_rx();
}

static inline void radio_abort_tx(void) {
    FASTSPI_STROBE(CC2420_SRXON);
#if ENERGEST_CONF_ON
    if (energest_current_mode[ENERGEST_TYPE_TRANSMIT]) {
        ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
        ENERGEST_ON(ENERGEST_TYPE_LISTEN);
    }
#endif /* ENERGEST_CONF_ON */
    radio_flush_rx();
}

static inline void radio_start_tx(void) {
    FASTSPI_STROBE(CC2420_STXON);
#if ENERGEST_CONF_ON
    ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
    ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
#endif /* ENERGEST_CONF_ON */
}

static inline void radio_write_tx(void) {
    FASTSPI_WRITE_FIFO(packet, packet_len_tmp - 1);
}

/* --------------------------- SFD interrupt ------------------------ */
interrupt(TIMERB1_VECTOR) __attribute__((section(".glossy")))
timerb1_interrupt(void) {
    // NOTE: if you modify the code if this function
    // you may need to change the constant part of the interrupt delay (currently 21 DCO ticks),
    // due to possible different compiler optimizations

    // compute the variable part of the delay with which the interrupt has been served
    T_irq = ((RTIMER_NOW_DCO() - TBCCR1) - 21) << 1;

    if (state == GLOSSY_STATE_RECEIVING && !SFD_IS_1) {
        // packet reception has finished
        // T_irq in [0,...,8]
        if (T_irq <= 8) {
            // NOPs (variable number) to compensate for the interrupt service delay (sec. 5.2)
            asm volatile("add %[d], r0" : : [d] "m" (T_irq));
            asm volatile("nop"); // irq_delay = 0
            asm volatile("nop"); // irq_delay = 2
            asm volatile("nop"); // irq_delay = 4
            asm volatile("nop"); // irq_delay = 6
            asm volatile("nop"); // irq_delay = 8
            // NOPs (fixed number) to compensate for HW variations (sec. 5.3)
            // (asynchronous MCU and radio clocks)
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
            // relay the packet
            radio_start_tx();
            // read TBIV to clear IFG
            tbiv = TBIV;
            glossy_end_rx();
        } else {
            // interrupt service delay is too high: do not relay the packet
            radio_flush_rx();
            state = GLOSSY_STATE_WAITING;
            // read TBIV to clear IFG
            tbiv = TBIV;
#if GLOSSY_DEBUG_STATUS
            debug_status[debug_status_index] = DEBUG_T_IRQ_HIGH;
            debug_status_index += 6;
#endif /* GLOSSY_DEBUG_STATUS */
        }
    } else {
        // read TBIV to clear IFG
        tbiv = TBIV;
        if (state == GLOSSY_STATE_WAITING && SFD_IS_1) {
            // packet reception has started
            glossy_begin_rx();
        } else {
            if (state == GLOSSY_STATE_RECEIVED && SFD_IS_1) {
                // packet transmission has started
                glossy_begin_tx();
            } else {
                if (state == GLOSSY_STATE_TRANSMITTING && !SFD_IS_1) {
                    // packet transmission has finished
                    glossy_end_tx();
                } else {
                    if (state == GLOSSY_STATE_ABORTED) {
                        // packet reception has been aborted
                        state = GLOSSY_STATE_WAITING;
                    } else {
                        if ((state == GLOSSY_STATE_WAITING) && (tbiv == TBIV_TBCCR4)) {
                            // initiator timeout
                            n_timeouts++;
                            if (rx_cnt == 0) {
#if GLOSSY_DEBUG_STATUS
                                debug_status[debug_status_index] = DEBUG_INITIATOR_TIMEOUT;
                                debug_status_index += 4;
#endif /* GLOSSY_DEBUG_STATUS */
                                // no packets received so far: send the packet again
                                tx_cnt = 0;
                                // set the packet length field to the appropriate value
                                fs_header_glossy->fs_len_field = packet_len_tmp;
                                // Increase the relay counter
                                fs_header_glossy->fs_relay_cnt_field = (3 << 6) | (n_timeouts * GLOSSY_INITIATOR_TIMEOUT);
                                // copy the application data to the data field
                                memcpy(&fs_header_glossy->fs_payload, data, (uint8_t) * data_len);
                                // set Glossy state
                                state = GLOSSY_STATE_RECEIVED;
                                // write the packet to the TXFIFO
                                radio_write_tx();
                                // start another transmission
                                radio_start_tx();
                                // schedule the timeout again
                                glossy_schedule_initiator_timeout();
                            } else {
                                // at least one packet has been received: just stop the timeout
                                glossy_stop_initiator_timeout();
                            }
                        } else {
                            if (tbiv == TBIV_TBCCR5) {
                                // rx timeout
                                if (state == GLOSSY_STATE_RECEIVING) {
                                    // we are still trying to receive a packet: abort the reception
                                    radio_abort_rx();
#if GLOSSY_DEBUG_STATUS
                                    debug_status[debug_status_index] = DEBUG_RECEIVER_TIMEOUT;
                                    debug_status_index += 6;
#endif /* GLOSSY_DEBUG_STATUS */
#if GLOSSY_DEBUG
                                    rx_timeout++;
#endif /* GLOSSY_DEBUG */
                                }
                                // stop the timeout
                                glossy_stop_rx_timeout();
                            } else {
                                if (state != GLOSSY_STATE_OFF) {
                                    // something strange is going on: go back to the waiting state
                                    radio_flush_rx();
                                    state = GLOSSY_STATE_WAITING;
#if GLOSSY_DEBUG_PINS
#if COOJA
                                    leds_off(LEDS_GREEN);
#endif /* COOJA */
#if FLOCKLAB
                                    UNSET_PIN_ADC6;
#endif /* FLOCKLAB */
#endif /* GLOSSY_DEBUG_PINS */
#if GLOSSY_DEBUG_STATUS
                                    debug_status[debug_status_index] = DEBUG_UNKNOWN;
                                    debug_status_index += 7;
#endif /* GLOSSY_DEBUG_STATUS */
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

/* --------------------------- Glossy process ----------------------- */
PROCESS(glossy_process, "Glossy busy-waiting process");

PROCESS_THREAD(glossy_process, ev, data) {
    PROCESS_BEGIN();

    packet = (uint8_t *) & packet_aligned[0];
    fs_header_glossy = (fs_header_glossy_struct *) & packet_aligned[0];
#if GLOSSY_DEBUG_PINS
#if FLOCKLAB
    INIT_PIN_ADC7_OUT;
    INIT_PIN_ADC6_OUT;
    INIT_PIN_ADC2_OUT;
#endif /* FLOCKLAB */
#endif /* GLOSSY_DEBUG_PINS */

    while (1) {
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_POLL);
        // prevent the Contiki main cycle to enter the LPM mode or
        // any other process to run while Glossy is running	
        while (GLOSSY_IS_ON() && RTIMER32_CLOCK_LT(RTIMER32_NOW(), t_stop));

#if COOJA
        // Cooja sometimes just does not want to stop transmitting
        while (state == GLOSSY_STATE_TRANSMITTING);
#endif /* COOJA */
#if GLOSSY_DEBUG_PINS
#if COOJA
        leds_off(LEDS_GREEN);
#endif /* COOJA */
#if FLOCKLAB
        UNSET_PIN_ADC6;
#endif /* FLOCKLAB */
#endif /* GLOSSY_DEBUG_PINS */
        // Glossy finished: execute the callback function
        dint();
        cb(rtimer, ptr);
        eint();
    }

    PROCESS_END();
}

static inline void glossy_disable_other_interrupts(void) {
    int s = splhigh();
    ie1 = IE1;
    ie2 = IE2;
    p1ie = P1IE;
    p2ie = P2IE;
    IE1 = 0;
    IE2 = 0;
    P1IE = 0;
    P2IE = 0;
    CACTL1 &= ~CAIE;
    DMA0CTL &= ~DMAIE;
    DMA1CTL &= ~DMAIE;
    DMA2CTL &= ~DMAIE;
    // disable etimer interrupts
    TACCTL1 &= ~CCIE;
    TBCCTL0 = 0;
    DISABLE_FIFOP_INT();
    CLEAR_FIFOP_INT();
    SFD_CAP_INIT(CM_BOTH);
    ENABLE_SFD_INT();
    // stop Timer B
    TBCTL = 0;
    // Timer B sourced by the DCO
    TBCTL = TBSSEL1;
    // start Timer B
    TBCTL |= MC1;
    splx(s);
    watchdog_stop();
}

static inline void glossy_enable_other_interrupts(void) {
    int s = splhigh();
    IE1 = ie1;
    IE2 = ie2;
    P1IE = p1ie;
    P2IE = p2ie;
    // enable etimer interrupts
    TACCTL1 |= CCIE;
#if COOJA
    if (TACCTL1 & CCIFG) {
        etimer_interrupt();
    }
#endif
    DISABLE_SFD_INT();
    CLEAR_SFD_INT();
    FIFOP_INT_INIT();
    ENABLE_FIFOP_INT();
    // stop Timer B
    TBCTL = 0;
    // Timer B sourced by the 32 kHz
    TBCTL = TBSSEL0;
    // start Timer B
    TBCTL |= MC1;
    splx(s);
    watchdog_start();
}

/* --------------------------- Main interface ----------------------- */
void glossy_start(uint8_t *data_, uint8_t *data_len_, uint8_t initiator_,
        uint8_t sync_, uint8_t tx_max_, rtimer32_clock_t t_stop_, rtimer_callback_t cb_,
        struct rtimer *rtimer_, void *ptr_) {
    // copy function arguments to the respective Glossy variables
    data = data_;
    data_len = data_len_;
    initiator = initiator_;
    sync = sync_;
    tx_max = tx_max_;
    //header = header_;
    t_stop = t_stop_;
    cb = cb_;
    rtimer = rtimer_;
    ptr = ptr_;
    // disable all interrupts that may interfere with Glossy
    glossy_disable_other_interrupts();
    // initialize Glossy variables
    // TODO: We should do that somewhere else!!!!
    memset(packet, 0, sizeof (packet_aligned));
    tx_cnt = 0;
    rx_cnt = 0;

    t_start = RTIMER_NOW_DCO();
    if ((uint8_t) * data_len) {
        // set Glossy packet length with relay counter
        packet_len_tmp = (uint8_t) * data_len + FOOTER_LEN + GLOSSY_RELAY_CNT_LEN;
        packet_len = packet_len_tmp;
        // set the packet length field to the appropriate value
        fs_header_glossy->fs_len_field = packet_len_tmp;
    } else {
        // packet length not known yet (only for receivers)
        packet_len = 0;
    }
    if (initiator) {
        // initiator: copy the application data to the data field
        memcpy(&fs_header_glossy->fs_payload, data, (uint8_t) * data_len);
        // set Glossy state
        state = GLOSSY_STATE_RECEIVED;
    } else {
        // receiver: set Glossy state
        state = GLOSSY_STATE_WAITING;
    }
    // set the relay_cnt field to 0
    fs_header_glossy->fs_relay_cnt_field = (3 << 6) | 0;
    // the reference time has not been updated yet
    t_ref_l_updated = 0;
#if !COOJA
    // resynchronize the DCO
    msp430_sync_dco();
#endif /* COOJA */

    // flush radio buffers
    radio_flush_rx();
    radio_flush_tx();
    if (initiator) {
        // write the packet to the TXFIFO
        radio_write_tx();
        // start the first transmission
        radio_start_tx();
#if GLOSSY_DEBUG_PINS
#if COOJA
        leds_on(LEDS_RED);
#endif /* COOJA */
#if FLOCKLAB
        SET_PIN_ADC7;
#endif /* FLOCKLAB */
#endif /* GLOSSY_DEBUG_PINS */
        // schedule the initiator timeout
        if (T_slot_h) {
            n_timeouts = 0;
            glossy_schedule_initiator_timeout();
        }
    } else {
        // turn on the radio
        radio_on();
#if GLOSSY_DEBUG_PINS
#if COOJA
        leds_on(LEDS_RED);
#endif /* COOJA */
#if FLOCKLAB
        SET_PIN_ADC7;
#endif /* FLOCKLAB */
#endif /* GLOSSY_DEBUG_PINS */
    }
    // activate the Glossy busy waiting process
    process_poll(&glossy_process);
}

uint8_t glossy_stop(void) {
    // stop the initiator timeout, in case it is still active
    glossy_stop_initiator_timeout();
    // turn off the radio
    radio_off();
#if GLOSSY_DEBUG_PINS
#if COOJA
    leds_off(LEDS_RED);
    leds_off(LEDS_BLUE);
    leds_off(LEDS_GREEN);
#endif /* COOJA */
#if FLOCKLAB
    UNSET_PIN_ADC7;
    UNSET_PIN_ADC6;
    UNSET_PIN_ADC2;
#endif /* FLOCKLAB */
#endif /* GLOSSY_DEBUG_PINS */
#if GLOSSY_DEBUG_STATUS
    debug_status[debug_status_index] = DEBUG_SRF_OFF;
    debug_status_index++;
#endif /* GLOSSY_DEBUG_STATUS */

    // flush radio buffers
    radio_flush_rx();
    radio_flush_tx();

    state = GLOSSY_STATE_OFF;
    // re-enable non Glossy-related interrupts
    glossy_enable_other_interrupts();
    // return the number of times the packet has been received
    return rx_cnt;
}

uint8_t *get_packet(void) {
    return packet;
}

uint8_t get_packet_len(void) {
    return packet_len_tmp;
}

uint8_t get_rx_cnt(void) {
    return rx_cnt;
}

uint8_t get_tx_cnt(void) {
    return tx_cnt;
}

uint8_t get_relay_cnt(void) {
    return relay_cnt;
}

rtimer_clock_t get_T_slot_h(void) {
    return T_slot_h;
}

uint8_t is_t_ref_l_updated(void) {
    return t_ref_l_updated;
}

rtimer_clock_t get_t_first_rx_l(void) {
    return t_first_rx_l;
}

rtimer_clock_t get_t_ref_l(void) {
    return t_ref_l;
}

void set_t_ref_l(rtimer_clock_t t) {
    t_ref_l = t;
}

void set_t_ref_l_updated(uint8_t updated) {
    t_ref_l_updated = updated;
}

uint8_t get_state(void) {
    return state;
}

static inline void estimate_slot_length(rtimer_clock_t t_rx_stop_tmp) {
    // estimate slot length if rx_cnt > 1
    // and we have received a packet immediately after our last transmission
    if ((rx_cnt > 1) && (fs_header_glossy->fs_relay_cnt_field == ((3 << 6) | (tx_relay_cnt_last + 2)))) {
        T_w_rt_h = t_tx_start - t_rx_stop;
        T_tx_h = t_tx_stop - t_tx_start;
        T_w_tr_h = t_rx_start - t_tx_stop;
        T_rx_h = t_rx_stop_tmp - t_rx_start;
        rtimer_clock_t T_slot_h_tmp = (T_tx_h + T_w_tr_h + T_rx_h + T_w_rt_h) / 2 - (packet_len * F_CPU) / 31250;
#if GLOSSY_SYNC_WINDOW
        T_slot_h_sum += T_slot_h_tmp;
        if ((++win_cnt) == GLOSSY_SYNC_WINDOW) {
            // update the slot length estimation
            T_slot_h = T_slot_h_sum / GLOSSY_SYNC_WINDOW;
            // halve the counters
            T_slot_h_sum /= 2;
            win_cnt /= 2;
        } else {
            if (win_cnt == 1) {
                // at the beginning, use the first estimation of the slot length
                T_slot_h = T_slot_h_tmp;
            }
        }
#else
        T_slot_h = T_slot_h_tmp;
#endif /* GLOSSY_SYNC_WINDOW */
    }
}

static inline void compute_sync_reference_time(void) {
#if COOJA
    rtimer_clock_t t_cap_l = RTIMER_NOW();
    rtimer_clock_t t_cap_h = RTIMER_NOW_DCO();
#else
    // capture the next low-frequency clock tick
    rtimer_clock_t t_cap_h, t_cap_l;
    CAPTURE_NEXT_CLOCK_TICK(t_cap_h, t_cap_l);
#endif /* COOJA */
    rtimer_clock_t T_rx_to_cap_h = t_cap_h - t_rx_start;
    unsigned long T_ref_to_rx_h = ((fs_header_glossy->fs_relay_cnt_field - 1) - (3 << 6)) * ((unsigned long) T_slot_h + (packet_len * F_CPU) / 31250);
    unsigned long T_ref_to_cap_h = T_ref_to_rx_h + (unsigned long) T_rx_to_cap_h;
    rtimer_clock_t T_ref_to_cap_l = 1 + T_ref_to_cap_h / CLOCK_PHI;
    // high-resolution offset of the reference time
    T_offset_h = (CLOCK_PHI - 1) - (T_ref_to_cap_h % CLOCK_PHI);
    // low-resolution value of the reference time
    t_ref_l = t_cap_l - T_ref_to_cap_l;
    // the reference time has been updated
    t_ref_l_updated = 1;
}

/* ----------------------- Interrupt functions ---------------------- */
inline void glossy_begin_rx(void) {
    t_rx_start = TBCCR1;
#if GLOSSY_DEBUG_PINS
#if COOJA
    leds_on(LEDS_GREEN);
#endif /* COOJA */
#if FLOCKLAB
    SET_PIN_ADC6;
#endif /* FLOCKLAB */
#endif /* GLOSSY_DEBUG_PINS */
#if GLOSSY_DEBUG_STATUS
    debug_status[debug_status_index] = DEBUG_BEGIN_RX_START;
    debug_status_index++;
#endif /* GLOSSY_DEBUG_STATUS */
    state = GLOSSY_STATE_RECEIVING;

    // Rx timeout: packet duration + 200 us
    // (packet duration: 32 us * packet_length, 1 DCO tick ~ 0.23 us)
    // HACK: We set the real timeout later
    t_rx_timeout = t_rx_start + (127 * 35 + 200) * 4;
    // wait until the FIFO pin is 1 (i.e., until the first byte is received)
    while (!FIFO_IS_1) {
        //if (packet_len && !RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
        if (!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
            radio_abort_rx();
#if GLOSSY_DEBUG_PINS
#if COOJA
            leds_off(LEDS_GREEN);
#endif /* COOJA */
#if FLOCKLAB
            UNSET_PIN_ADC6;
#endif /* FLOCKLAB */
#endif /* GLOSSY_DEBUG_PINS */
#if GLOSSY_DEBUG
            rx_timeout++;
#endif /* GLOSSY_DEBUG */
#if GLOSSY_DEBUG_STATUS
            debug_status[debug_status_index] = DEBUG_BEGIN_RX_TIMEOUT_FIRST_BYTE;
            debug_status_index = +7;
#endif /* GLOSSY_DEBUG_STATUS */
            return;
        }
    };
#if COOJA
    // OL: do not ask why
    // MB: We have to wait because there is no packet in the buffer, yet. It's a bug.
    uint8_t i;
    for (i = 0; i < 20; i++) {
        asm volatile("nop");
    }
#endif /* COOJA */
    // read the first byte (i.e., the len field) from the RXFIFO
    FASTSPI_READ_FIFO_BYTE(fs_header_glossy->fs_len_field);
    // keep receiving only if it has the right length
    if ((fs_header_glossy->fs_len_field < FOOTER_LEN)
            || (fs_header_glossy->fs_len_field > 127)) {
        // packet with a wrong length: abort packet reception
        radio_abort_rx();
#if GLOSSY_DEBUG_PINS
#if COOJA
        leds_off(LEDS_GREEN);
#endif /* COOJA */
#if FLOCKLAB
        UNSET_PIN_ADC6;
#endif /* FLOCKLAB */
#endif /* GLOSSY_DEBUG_PINS */
#if GLOSSY_DEBUG
        bad_length++;
#endif /* GLOSSY_DEBUG */
#if GLOSSY_DEBUG_STATUS
        debug_status[debug_status_index] = DEBUG_BEGIN_RX_BAD_LENGHT;
        debug_status_index += 6;
#endif /* GLOSSY_DEBUG_STATUS */
        return;
    }
    bytes_read = 1;
    packet_len_tmp = fs_header_glossy->fs_len_field;
    t_rx_timeout = t_rx_start + ((rtimer_clock_t) packet_len_tmp * 35 + 200) * 4;

    if (packet_len_tmp > 8) {
        // if packet is longer than 8 bytes, read all bytes but the last 8
        while (bytes_read <= packet_len_tmp - 8) {
            // wait until the FIFO pin is 1 (until one more byte is received)
#if !COOJA
            while (!FIFO_IS_1) {
                if (!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
                    radio_abort_rx();
#if GLOSSY_DEBUG_PINS
#if COOJA
                    leds_off(LEDS_GREEN);
#endif /* COOJA */
#if FLOCKLAB
                    UNSET_PIN_ADC6;
#endif /* FLOCKLAB */
#endif /* GLOSSY_DEBUG_PINS */
#if GLOSSY_DEBUG
                    rx_timeout++;
#endif /* GLOSSY_DEBUG */
#if GLOSSY_DEBUG_STATUS
                    debug_status[debug_status_index] = DEBUG_BEGIN_RX_TIMEOUT_REST_BYTE;
                    debug_status_index += 7;
#endif /* GLOSSY_DEBUG_STATUS */
                    return;
                }
            };
#endif /* ! COOJA */
            // read another byte from the RXFIFO
            FASTSPI_READ_FIFO_BYTE(packet[bytes_read]);
            bytes_read++;
        }
    }
    glossy_schedule_rx_timeout();
#if GLOSSY_DEBUG_STATUS
    debug_status[debug_status_index] = DEBUG_BEGIN_RX_STOP;
    debug_status_index++;
#endif /* GLOSSY_DEBUG_STATUS */
}

inline void glossy_end_rx(void) {
    rtimer_clock_t t_rx_stop_tmp = TBCCR1;
#if GLOSSY_DEBUG_PINS
#if COOJA
    leds_off(LEDS_GREEN);
#endif /* COOJA */
#if FLOCKLAB
    UNSET_PIN_ADC6;
#endif /* FLOCKLAB */
#endif /* GLOSSY_DEBUG_PINS */
#if GLOSSY_DEBUG_STATUS
    debug_status[debug_status_index] = DEBUG_END_RX_START;
    debug_status_index++;
#endif /* GLOSSY_DEBUG_STATUS */
    // read the remaining bytes from the RXFIFO
    FASTSPI_READ_FIFO_NO_WAIT(&packet[bytes_read], packet_len_tmp - bytes_read + 1);
    bytes_read = packet_len_tmp + 1;

    if ((GLOSSY_CRC_FIELD & FOOTER1_CRC_OK) && ((fs_header_glossy->fs_relay_cnt_field) - (3 << 6)) >= 0) {
        // packet correctly received
        // increment relay_cnt field
        fs_header_glossy->fs_relay_cnt_field++;
        if (tx_cnt == tx_max) {
            // no more Tx to perform: stop Glossy
            radio_off();
            state = GLOSSY_STATE_OFF;
#if GLOSSY_DEBUG_PINS
#if COOJA
            leds_off(LEDS_RED);
            leds_off(LEDS_GREEN);
            leds_off(LEDS_BLUE);
#endif /* COOJA */
#if FLOCKLAB
            UNSET_PIN_ADC7;
            UNSET_PIN_ADC6;
            UNSET_PIN_ADC2;
#endif /* FLOCKLAB */
#endif /* GLOSSY_DEBUG_PINS */
#if GLOSSY_DEBUG_STATUS
            debug_status[debug_status_index] = DEBUG_END_RX_TX_MAX;
            debug_status_index++;
            debug_status[debug_status_index] = DEBUG_SRF_OFF;
            debug_status_index++;
#endif /* GLOSSY_DEBUG_STATUS */
        } else {
            // write Glossy packet to the TXFIFO
            radio_write_tx();
            state = GLOSSY_STATE_RECEIVED;
        }
        if (rx_cnt == 0) {
            // first successful reception:
            // store current time, received relay counter,
            t_first_rx_l = RTIMER_NOW();
            relay_cnt = (fs_header_glossy->fs_relay_cnt_field - 1) - (3 << 6);
        }
        rx_cnt++;
        if (sync) {
            estimate_slot_length(t_rx_stop_tmp);
        }
        t_rx_stop = t_rx_stop_tmp;
        if (initiator) {
            // a packet has been successfully received: stop the initiator timeout
            glossy_stop_initiator_timeout();
        }
        //if (!packet_len) {
        packet_len = packet_len_tmp;
        *data_len = packet_len_tmp - FOOTER_LEN - GLOSSY_RELAY_CNT_LEN;
        //}
#if GLOSSY_DEBUG_STATUS
        debug_status[debug_status_index] = DEBUG_END_RX_STOP;
        debug_status_index++;
#endif /* GLOSSY_DEBUG_STATUS */
    } else {
#if GLOSSY_DEBUG
        bad_crc++;
#endif /* GLOSSY_DEBUG */
        // packet corrupted, abort the transmission before it actually starts
        radio_abort_tx();
        state = GLOSSY_STATE_WAITING;
#if GLOSSY_DEBUG_STATUS
        debug_status[debug_status_index] = DEBUG_END_RX_BAD_CRX;
        debug_status_index += 5;
#endif /* GLOSSY_DEBUG_STATUS */
    }
}

inline void glossy_begin_tx(void) {
    t_tx_start = TBCCR1;
#if GLOSSY_DEBUG_PINS
#if COOJA
    leds_on(LEDS_BLUE);
#endif /* COOJA */
#if FLOCKLAB
    SET_PIN_ADC2;
#endif /* FLOCKLAB */
#endif /* GLOSSY_DEBUG_PINS */
#if GLOSSY_DEBUG_STATUS
    debug_status[debug_status_index] = DEBUG_BEGIN_TX_START;
    debug_status_index++;
#endif /* GLOSSY_DEBUG_STATUS */
    state = GLOSSY_STATE_TRANSMITTING;
    tx_relay_cnt_last = fs_header_glossy->fs_relay_cnt_field - (3 << 6);
    // copy the application data from the data field
    memcpy(data, &fs_header_glossy->fs_payload, (uint8_t) * data_len);

    if ((T_slot_h) && (!t_ref_l_updated) && (rx_cnt)) {
        // compute the reference time after the first reception (higher accuracy)
        compute_sync_reference_time();
    }
#if GLOSSY_DEBUG_STATUS
    debug_status[debug_status_index] = DEBUG_BEGIN_TX_STOP;
    debug_status_index++;
#endif /* GLOSSY_DEBUG_STATUS */
}

inline void glossy_end_tx(void) {
    ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
    ENERGEST_ON(ENERGEST_TYPE_LISTEN);
    t_tx_stop = TBCCR1;
#if GLOSSY_DEBUG_PINS
#if COOJA
    leds_off(LEDS_BLUE);
#endif /* COOJA */
#if FLOCKLAB
    UNSET_PIN_ADC2;
#endif /* FLOCKLAB */
#endif /* GLOSSY_DEBUG_PINS */
#if GLOSSY_DEBUG_STATUS
    debug_status[debug_status_index] = DEBUG_END_TX_START;
    debug_status_index++;
#endif /* GLOSSY_DEBUG_STATUS */
    // stop Glossy if tx_cnt reached tx_max (and tx_max > 1 at the initiator)
    if ((++tx_cnt == tx_max) && ((tx_max - initiator) > 0)) {
        radio_off();
#if GLOSSY_DEBUG_PINS
#if COOJA
        leds_off(LEDS_RED);
#endif /*COOJA*/
#if FLOCKLAB
        UNSET_PIN_ADC7;
#endif /* FLOCKLAB */
#endif /* GLOSSY_DEBUG_PINS */
#if GLOSSY_DEBUG_STATUS
        debug_status[debug_status_index] = DEBUG_END_TX_TX_MAX;
        debug_status_index++;
        debug_status[debug_status_index] = DEBUG_SRF_OFF;
        debug_status_index++;
#endif /* GLOSSY_DEBUG_STATUS */
        state = GLOSSY_STATE_OFF;
    } else {
        state = GLOSSY_STATE_WAITING;
    }
    radio_flush_tx();
#if GLOSSY_DEBUG_STATUS
    debug_status[debug_status_index] = DEBUG_END_TX_STOP;
    debug_status_index++;
#endif /* GLOSSY_DEBUG_STATUS */
}

void glossy_reset(void) {
    rx_cnt = 0;
    tx_cnt = 0;
}

/* ------------------------------ Timeouts -------------------------- */
inline void glossy_schedule_rx_timeout(void) {
    TBCCR5 = t_rx_timeout;
    TBCCTL5 = CCIE;
}

inline void glossy_stop_rx_timeout(void) {
    TBCCTL5 = 0;
}

inline void glossy_schedule_initiator_timeout(void) {
    //#if !COOJA
    TBCCR4 = t_start + (n_timeouts + 1) * GLOSSY_INITIATOR_TIMEOUT * ((unsigned long) T_slot_h + (packet_len * F_CPU) / 31250);
    TBCCTL4 = CCIE;
    //#endif
}

inline void glossy_stop_initiator_timeout(void) {
    TBCCTL4 = 0;
}
