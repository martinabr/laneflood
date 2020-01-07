/*
 * Copyright (c) 2016, TU Dresden.
 * Copyright (c) 2011, ETH Zurich.
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
 *         Federico Ferrari <ferrari@tik.ee.ethz.ch>
 *
 */

#include "laneflood.h"

static fs_header_ptp_struct *fs_header_ptp; /**< \brief Forwarder selection header*/
static fs_data_struct *fs_data;
static fs_sync_struct *fs_sync;

static struct rtimer rt; /**< \brief Rtimer used to schedule Glossy. */
static struct pt pt; /**< \brief Protothread used to schedule Glossy. */
static rtimer32_clock_t t_ref_l_old = 0; /**< \brief Reference time computed from the Glossy
                                                phase before the last one. \sa get_t_ref_l */
static uint8_t sessions_without_sync = 0;

static uint8_t sync_missed = 0; /**< \brief Current number of consecutive phases without
                                                synchronization (reference time not computed). */
static rtimer32_clock_t t_start = 0; /**< \brief Starting time (low-frequency clock)
                                                of the last Glossy phase. */
static int period_skew = 0; /**< \brief Current estimation of clock skew over a period
                                                of length \link GLOSSY_PERIOD \endlink. */


static unsigned long packets_received = 0; /**< \brief Current number of received packets. */
static unsigned long packets_missed = 0; /**< \brief Current number of missed packets. */
static unsigned long packets_sleep = 0;
static unsigned long latency = 0; /**< \brief Latency of last Glossy phase, in us. */
//static unsigned long sum_latency = 0;      /**< \brief Current sum of latencies, in ticks of low-frequency
//                                               clock (used to compute average). */

static uint8_t skew_estimated = 0; /**< \brief Not zero if the clock skew over a period of length
                                                \link GLOSSY_PERIOD \endlink has already been estimated. */


static fs_header_common_struct *fs_header_common;

static uint32_t data_buf[128 / sizeof (uint32_t)] = {0}; /**< \brief The packet buffer */

static struct pt fs_pt; /**< \brief Protothread used to schedule FS. */

static uint8_t sender = 0;
static uint8_t last_sender = 0;
static uint16_t seq_no_to_sync = 0;
static uint16_t cur_seq_no = 0;
static rtimer32_clock_t t_heartbeat_start = 0;
static rtimer32_clock_t fs_period = GLOSSY_PERIOD;
static uint8_t data_len;

static rtimer32_clock_t delay;

static rtimer32_clock_t t_stop;

// Forwarder selection
static uint8_t fs_state = FS_STATE_OFF;
static uint8_t fs_prev_state = FS_STATE_OFF;
static rtimer32_clock_t fs_t_stop;

static uint8_t forwarder;
static uint8_t sync_flag;

static uint8_t fs_timeout_cnt = 0;

static unsigned long rounds = 0;

static NODE_ID_TYPE current_initiator_node_id = INITIATOR_NODE_ID;
static NODE_ID_TYPE destination_id;

static struct etimer et;

static uint8_t fs_header;
static uint16_t fs_seq_no;
static NODE_ID_TYPE fs_src;
static NODE_ID_TYPE fs_dest;
static uint8_t fs_dist;
static uint8_t fs_slack;
static uint8_t fs_flags;

struct packetqueue_item *laneflood_packetqueue_item;
struct queuebuf *laneflood_queuebuf_item;

static uint8_t send_data = 0;
static uint8_t flip_a_coin = 0;

/**
 * State transition table is used to represent the operation of the
 * protocol by a number of cooperating state machines each comprising a
 * group of connected, mutually exclusive states. Only one state of each
 * machine can be active at any given time. By invoking the table with a state
 * the corresponding callback function is called. */
const sm_action laneflood_table[NUM_ROLES][NUM_STATES] ={
    { fs_cb_setup_sync_receiver,
        fs_cb_response_receiver,
        fs_cb_data_receiver,
        fs_cb_wait,
        fs_cb_sleep,},
    { fs_cb_setup_sync_sender,
        fs_cb_response_sender,
        fs_cb_data_sender,
        fs_cb_wait,
        fs_cb_sleep}
};

PROCESS(glossy_print_stats_process, "Glossy print stats");

PROCESS_THREAD(glossy_print_stats_process, ev, data) {
    unsigned long avg_rel = 0;
    unsigned long avg_radio_on = 0;
    //unsigned long avg_latency = 0;

    PROCESS_BEGIN();

    while (1) {
        // Before we print laneflood and glossy statistics
        // we push the data to the application layer
        PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
        if ((fs_header != GLOSSY_HEADER) && get_rx_cnt()
                && (fs_header != FS_SYNC_HEADER)
                && ((node_id == fs_src) || (node_id == fs_dest))) {
            NETSTACK_LLSEC.input();
        }

        /** calculate **/
        if (GLOSSY_IS_BOOTSTRAPPING()) {
            if (get_rx_cnt()) {
                // Increment number of successfully received packets.
                packets_received++;
                if (packets_missed + packets_sleep + packets_received < fs_seq_no) {
                    // Might be that we missed a round at the beginning
                    packets_sleep++;
                }
            }
            if (get_rx_cnt() || rounds != 0) {
                rounds = packets_missed + packets_received + packets_sleep;
            }
        } else {
            if (get_rx_cnt()) {
                // Increment number of successfully received packets.
                packets_received++;
                // Compute current average reliability.
                avg_rel = packets_received * 1e5 / (packets_received + packets_missed);
#if ENERGEST_CONF_ON
                // Compute average radio-on time, in microseconds.
                // GLOSSY_PERIOD = 32768 ticks/s * 1s/4 = 131 072 ticks
                // GLOSSY_PERIOD ticks / 32768 ticks/s = 1/4 s
                // Rest is ratio (ticks / ticks)
                // Energest is measured over a period of GLOSSY_PERIOD
                avg_radio_on = (unsigned long) GLOSSY_PERIOD * 1e6 / RTIMER_SECOND *
                        (energest_type_time(ENERGEST_TYPE_LISTEN) + energest_type_time(ENERGEST_TYPE_TRANSMIT)) /
                        (energest_type_time(ENERGEST_TYPE_CPU) + energest_type_time(ENERGEST_TYPE_LPM));
#endif /* ENERGEST_CONF_ON */
                // Compute latency during last Glossy phase.
                rtimer_clock_t lat = get_t_first_rx_l() - get_t_ref_l();
                // Add last latency to sum of latencies.
                //sum_latency += lat;
                // Compute average latency, in microseconds.
                //avg_latency = sum_latency * 1e6 / (RTIMER_SECOND * packets_received);
                // Convert latency to microseconds.
                latency = (unsigned long) (lat) * 1e6 / RTIMER_SECOND;
            } else {
                if (rounds != 0 && (fs_state == FS_STATE_SLEEP)) {
                    // We have not missed a packet when
                    // we are in sleep state or
                    // we are in wait state and have not received a packet
                    packets_sleep++;
                } else {
                    // Increment number of missed packets.
                    packets_missed++;
                }
            }
            if (get_rx_cnt() || rounds != 0) {
                rounds = packets_missed + packets_received + packets_sleep;
            }
        }

        /** print **/
        if (GLOSSY_IS_BOOTSTRAPPING()) {
            printf("r3sa%uro%lu", fs_prev_state, rounds);
            if (get_rx_cnt()) {
                printf("r1\n");
            } else {
                printf("r0\n");
            }
        } else {
#if GLOSSY_DEBUG_STATUS
            if (fs_prev_state != FS_STATE_SLEEP) {
                uint8_t i;
                printf("st");
                for (i = 0; i < debug_status_index; i++) {
                    printf("%02u", debug_status[i]);
                }
                printf("\n");
                debug_status_index = 0;
                memset(debug_status, DEBUG_WAITING, sizeof (debug_status));
            }
#endif /* GLOSSY_DEBUG_STATUS */
            // Print general LaneFlood information
            printf("sa%use%ufo%uro%lu\n", fs_prev_state, last_sender, forwarder, rounds);
            if (get_rx_cnt()) {
                // laneflood stuff
                if (fs_state == FS_STATE_OFF || (fs_prev_state == FS_STATE_OFF && fs_state == FS_STATE_SETUP_SYNC)) {
                    printf("r4");
                } else {
                    printf("r1");
                }
                printf("he%uno%usr%ude%udi%usl%u",
                        fs_header, fs_seq_no, fs_src, fs_dest, fs_dist, fs_slack);
                // glossy stuff
                // Print information about last packet and related latency.
                printf("rx%utx%ure%upl%lu.%03lu",
                        get_rx_cnt(), get_tx_cnt(), get_relay_cnt() + 1, latency / 1000, latency % 1000);
                // Print information about average reliability.
                printf("py%lu.%03lupm%lupt%lu\n",
                        avg_rel / 1000, avg_rel % 1000, packets_missed, packets_received + packets_missed);
                // Print information about average latency.
                //						printf("pa%lu.%03lu\n",
                //								avg_latency / 1000, avg_latency % 1000);
#if ENERGEST_CONF_ON
                // Print information about average radio-on time.
                printf("ra%lu.%03lu\n",
                        avg_radio_on / 1000, avg_radio_on % 1000);
#endif /* ENERGEST_CONF_ON */
#if GLOSSY_DEBUG
                if ((node_id != INITIATOR_NODE_ID) && (fs_header == FS_SYNC_HEADER)) {
                    printf("sk%ld", (long) (period_skew * 1e6) / GLOSSY_PERIOD);
                }
                printf("ir%urt%ubl%ubc%ubh%u\n",
                        high_T_irq, rx_timeout, bad_length, bad_crc, bad_header);
#endif /* GLOSSY_DEBUG */
            } else {
                if (rounds != 0 && (fs_state == FS_STATE_SLEEP)) {
                    printf("r2\n");
                } else {
                    // Print failed reception.
                    printf("r0\n");
                }
            }
        }
        // Reset glossy rx and tx cnt
        glossy_reset();
    }
    PROCESS_END();
}

void set_fs_state(uint8_t state) {
    fs_prev_state = fs_state;
    fs_state = state;
}

/* ---------------------- Node distance functions ------------------- */
static struct dist_info* dist_head = NULL;
MEMB(dists, struct dist_info, MAX_DISTANCES_COUNT + 1);

static inline uint8_t
dist_known(NODE_ID_TYPE node) {
    struct dist_info* result;
    DL_SEARCH_SCALAR(dist_head, result, id, node);
    return result ? result->dist : 0;
}

struct dist_info*
lookup_dist_info(NODE_ID_TYPE node) {
    struct dist_info* result;
    DL_SEARCH_SCALAR(dist_head, result, id, node);
    return result;
}

uint8_t
get_dist(NODE_ID_TYPE node) {
    return dist_known(node);
}

struct dist_info* get_dist_head() {
    return dist_head;
}

void
set_dist(NODE_ID_TYPE node, uint8_t distance) {
    struct dist_info* new_dist;
    struct dist_info* result;
    DL_SEARCH_SCALAR(dist_head, result, id, node);
    if (result) {
        new_dist = result;
        DL_DELETE(dist_head, result);
    } else {
        new_dist = memb_alloc(&dists);
        if (!new_dist) {
            printf("Out of memory.\n");
        }
    }
    new_dist->id = node;
    new_dist->dist = distance;
    DL_APPEND(dist_head, new_dist);
}

void
shrink_dists() {
    struct dist_info *item;
    uint16_t elem_count;
    DL_COUNT(dist_head, item, elem_count);
    while (elem_count-- > MAX_DISTANCES_COUNT) {
        item = dist_head->prev;
        DL_DELETE(dist_head, item);
        memb_free(&dists, item);
    }
}

static inline void
estimate_period_skew(void) {
    // Estimate clock skew over a period only if the reference time has been updated.
    if (GLOSSY_IS_SYNCED()) {
        // Estimate clock skew based on previous reference time and the Glossy period.
        period_skew = get_t_ref_l() - (t_ref_l_old + (rtimer_clock_t) GLOSSY_PERIOD);
        // Update old reference time with the newer one.
        t_ref_l_old = get_t_ref_l();
        // If Glossy is still bootstrapping, count the number of consecutive updates of the reference time.
        if (GLOSSY_IS_BOOTSTRAPPING()) {
            // Increment number of consecutive updates of the reference time.
            skew_estimated++;
            // Check if Glossy has exited from bootstrapping.
            if (!GLOSSY_IS_BOOTSTRAPPING()) {
                // Glossy has exited from bootstrapping.
                // Initialize Energest values.
                energest_init();
#if GLOSSY_DEBUG
                high_T_irq = 0;
                bad_crc = 0;
                bad_length = 0;
#endif /* GLOSSY_DEBUG */
                if (IS_INITIATOR()) {
                    // Store the reference seq-no for the heartbeat
                    seq_no_to_sync = fs_seq_no + FS_INIT_SYNC_GUARD + 1;
                }
            }
        }
    }
}

static inline void
estimate_fs_period_skew(rtimer32_clock_t period) {
    // Estimate clock skew over a period only if the reference time has been updated.
    if (GLOSSY_IS_SYNCED()) {
        // Estimate clock skew based on previous reference time and the Glossy period.
        period_skew = (TIME_16BIT_TO_32BIT(get_t_ref_l()) - (t_ref_l_old + period));
        // Update old reference time with the newer one.
        t_ref_l_old = TIME_16BIT_TO_32BIT(get_t_ref_l());
        sessions_without_sync = 0;
    }
}

void
fs_write_header_common(fs_header_common_struct *fs_header_common_, uint8_t header_, uint16_t seq_no_) {
    fs_header_common_->fs_header_field = header_;
    fs_header_common_->fs_seq_no = uip_htons(seq_no_);
}

void
fs_read_header_common(fs_header_common_struct *fs_header_common_) {
    fs_header = fs_header_common_->fs_header_field;
    fs_seq_no = uip_htons(fs_header_common_->fs_seq_no);
}

void
fs_read_sync(fs_sync_struct *fs_sync_) {
    seq_no_to_sync = uip_htons(fs_sync_->fs_seq_no_to_sync);
}

void
fs_write_header_ptp(fs_header_ptp_struct *fs_header_ptp_, NODE_ID_TYPE src_, NODE_ID_TYPE dest_, uint8_t dist_, uint8_t slack_) {
    fs_src = fs_header_ptp_->fs_src_field = src_;
    fs_dest = fs_header_ptp_->fs_dest_field = dest_;
    fs_dist = fs_header_ptp_->fs_dist_field = dist_;
    fs_slack = fs_header_ptp_->fs_slack_field = slack_;
}

void
fs_read_header_ptp(fs_header_ptp_struct *fs_header_) {
    fs_src = fs_header_->fs_src_field;
    fs_dest = fs_header_->fs_dest_field;
    fs_dist = fs_header_->fs_dist_field;
    fs_slack = fs_header_->fs_slack_field;
}

uint8_t
fs_write_data(fs_data_struct *fs_data_, uint8_t header_len_, uint8_t flags_, uint8_t *buffer_, uint8_t buffer_len_) {
    fs_flags = fs_data_->fs_data_flags = flags_;
    // Check if the data fits in the packet
    int8_t packet_len_tmp = sizeof (data_buf) - 1 - FS_GLOSSY_HEADER_LEN - header_len_ - buffer_len_;
    if (packet_len_tmp >= 0) {
        memcpy(&fs_data_->fs_payload[0], buffer_, buffer_len_);
        return buffer_len_;
    } else {
        printf("[ERROR] Packet to big");
    }
    return 0;
}

uint8_t
fs_read_data(fs_data_struct *fs_data_, uint8_t *buffer_) {
    int len = data_len - FS_HEADER_COMMON_LEN - FS_HEADER_PTP_LEN - FS_DATA_LEN;
    if (len > 0) {
        fs_flags = fs_data_->fs_data_flags;
        memcpy(buffer_, fs_data_->fs_payload, len);
        return len;
    } else {
        printf("[ERROR]: Packet to small \n");
        return 0;
    }
}

PT_THREAD(fs_cb_setup_sync_sender(struct pt *pt_, struct rtimer *t_, void *ptr_)) {
    PT_BEGIN(pt_);
    forwarder = 1;
    // Schedule end of Glossy phase based on GLOSSY_DURATION.
    t_stop = RTIMER32_TIME(t_) + GLOSSY_DURATION;
    if (send_data) {
        // We have application data
        // Set source and destination
        destination_id = packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[7];
        current_initiator_node_id = node_id;
        // Write and initialise, the common header and the ptp header and cpy application data
        fs_write_header_common(fs_header_common, FS_SETUP_HEADER, ++fs_seq_no);
        fs_header_ptp = (fs_header_ptp_struct *) & fs_header_common->fs_next_field[0];
        fs_write_header_ptp(fs_header_ptp, node_id, destination_id, 0, FS_INIT_SLACK);
        fs_data = (fs_data_struct *) & fs_header_ptp->fs_next_field[0];
        data_len = FS_HEADER_COMMON_LEN + FS_HEADER_PTP_LEN + FS_DATA_LEN;
        uint8_t app_data_tmp = fs_write_data(fs_data, data_len, 0, (uint8_t *) packetbuf_dataptr(), packetbuf_datalen());
        data_len += app_data_tmp;
        while (RTIMER_CLOCK_LT(RTIMER32_NOW() - delay, FS_PACKET_CREATION_DELAY_SENDER));
        // Start Glossy.
        glossy_start((uint8_t *) & data_buf[0], &data_len, GLOSSY_INITIATOR, 0, N_TX, t_stop,
                (rtimer_callback_t) fs_scheduler, t_, ptr_);
    } else {
        // We sync the network
        if (node_id == INITIATOR_NODE_ID) {
            // Fill the common header
            fs_write_header_common(fs_header_common, FS_SYNC_HEADER, ++fs_seq_no);
            data_len = FS_HEADER_COMMON_LEN;
            while (RTIMER_CLOCK_LT(RTIMER32_NOW() - delay, FS_PACKET_CREATION_DELAY_SENDER));
            // Start Glossy with the glossy packet
            glossy_start((uint8_t *) & data_buf[0], &data_len, GLOSSY_INITIATOR, 1, N_TX, t_stop,
                    (rtimer_callback_t) fs_scheduler, t_, ptr_);
        } else {
            // We have application data
            // Set source and destination
            fs_dest = packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[7];
            fs_write_header_common(fs_header_common, FS_SETUP_HEADER, ++fs_seq_no);
            fs_header_ptp = (fs_header_ptp_struct *) & fs_header_common->fs_next_field[0];
            fs_write_header_ptp(fs_header_ptp, node_id, fs_dest, 0, FS_INIT_SLACK);
            fs_data = (fs_data_struct *) & fs_header_ptp->fs_next_field[0];
            data_len = FS_HEADER_COMMON_LEN + FS_HEADER_PTP_LEN + FS_DATA_LEN;
            uint8_t app_data_tmp = fs_write_data(fs_data, data_len, 0, packetbuf_dataptr(), packetbuf_datalen());
            data_len += app_data_tmp;
            while (RTIMER_CLOCK_LT(RTIMER32_NOW() - delay, FS_PACKET_CREATION_DELAY_SENDER));
            // Start Glossy.
            glossy_start((uint8_t *) & data_buf[0], &data_len, GLOSSY_INITIATOR, 0, N_TX, t_stop,
                    (rtimer_callback_t) fs_scheduler, t_, ptr_);
        }
    }
    // Yield the protothread. It will be resumed when Glossy terminates.
    PT_YIELD(pt_);

    // Stop Glossy.
    glossy_stop();
    if (get_rx_cnt()) {
        fs_timeout_cnt = 0;
        // Read the common header to know what was actually transmitted in the network
        fs_read_header_common(fs_header_common);
        // We are done with synching
        // Change state and set the new sender if necessary
        if (fs_header == FS_SYNC_HEADER) {
            last_sender = sender;
            set_fs_state(FS_STATE_WAIT);
        } else {
            if (fs_header == FS_SETUP_HEADER) {
                // remove packet from queue when received application data is the same as sent
                fs_header_ptp = (fs_header_ptp_struct *) & fs_header_common->fs_next_field[0];
                fs_read_header_ptp(fs_header_ptp);
                // make sure, that this is our paket
                if (fs_src == node_id) {
                    //printf("delete packet \n");
                    packetqueue_dequeue(&tx_queue); // Remove the item
                    // Source node has distance 0 to itself.
                    set_dist(fs_src, 0);
                } else {
                    // Someone else was sending a SETUP message
                    destination_id = fs_dest;
                    fs_data = (fs_data_struct *) & fs_header_ptp->fs_next_field[0];
                    int ret = fs_read_data(fs_data, packetbuf_dataptr());
                    packetbuf_set_datalen(ret);
                    set_dist(fs_src, get_relay_cnt() + 1);
                }
                last_sender = sender;
                set_fs_state(FS_STATE_RESPONSE);
            } else {
                set_fs_state(FS_STATE_SETUP_SYNC);
            }
        }
    } else {
        fs_timeout_cnt++;
        if (fs_timeout_cnt < FS_TIMEOUT_SENDER) {
            set_fs_state(FS_STATE_SETUP_SYNC);
        } else {
            set_fs_state(FS_STATE_SLEEP);
        }
    }
    if (node_id == INITIATOR_NODE_ID && fs_t_stop == t_heartbeat_start + FS_PERIOD) {
        // We Should check, whether we are here for the first time of a round
        // Calculate the next heartbeat after the SYNC or SETUP message
        t_heartbeat_start = fs_t_stop;
    }
    // Schedule the next time the node either starts Glossy or checks the time
    rtimer32_set(t_, RTIMER32_TIME(t_) + GLOSSY_PERIOD, 1, (rtimer_callback_t) fs_scheduler, ptr_);
    // Print statistics
    process_poll(&glossy_print_stats_process);
    // Yield the protothread.
    PT_YIELD(pt_);
    PT_END(pt_);
}

PT_THREAD(fs_cb_setup_sync_receiver(struct pt *pt_, struct rtimer *t_, void *ptr_)) {
    PT_BEGIN(pt_);
    forwarder = 1;
    // Schedule end of Glossy phase based on GLOSSY_DURATION.
    t_stop = RTIMER32_TIME(t_) + GLOSSY_DURATION;
    data_len = 0;
    while (RTIMER_CLOCK_LT(RTIMER32_NOW() - delay, FS_PACKET_CREATION_DELAY_RECEIVER));
    glossy_start((uint8_t *) & data_buf[0], &data_len, GLOSSY_RECEIVER, 1,
            N_TX, t_stop, (rtimer_callback_t) fs_scheduler, t_, ptr_);
    // Yield the protothread. It will be resumed when Glossy terminates.
    PT_YIELD(pt_);

    // Stop Glossy.
    glossy_stop();
    if (get_rx_cnt()) {
        fs_timeout_cnt = 0;
        // Read the common header
        fs_read_header_common(fs_header_common);
        // We received the expected headers?
        if (fs_header == FS_SYNC_HEADER) {
            // We received the FS_SYNC_HEADER
            // Synchronise to the initiator
            // (but not in the first round after entering the heartbeat)
            if (!GLOSSY_IS_SYNCED()) {
                // The reference time was not updated:
                // increment reference time by the last time we received a SYNC packet + period_skew.
                set_t_ref_l(GLOSSY_REFERENCE_TIME + ((fs_period * sessions_without_sync)) + period_skew);
                set_t_ref_l_updated(1);
                // Increment sync_missed.
                sync_missed++;
            } else {
                // The reference time was not updated: reset sync_missed to zero.
                sync_missed = 0;
            }
            // Estimate the clock skew over the last period.
            estimate_fs_period_skew(fs_period * sessions_without_sync);
            if (fs_t_stop == t_heartbeat_start + FS_PERIOD) {
                // We are here for the first time in this session
                // Set the new heartbeat
                t_heartbeat_start = fs_t_stop;
            }
            if (GLOSSY_IS_SYNCED() && fs_period == FS_PERIOD) {
                // We are done with bootstrapping
                // Adjust the start of the heartbeat according to the period_skew
                t_heartbeat_start += period_skew;
            }
            // When we enter the heartbeat for the first time, fs_period is set to GLOSSY_PERIOD.
            // We can now change it to the lower frequency FS_PERIOD
            if (fs_period == GLOSSY_PERIOD) {
                fs_period = FS_PERIOD;
                rtimer32_set(t_, RTIMER32_TIME(t_) + GLOSSY_PERIOD, 1,
                        (rtimer_callback_t) fs_scheduler, ptr_);
            } else {
                fs_period = FS_PERIOD;
                // Schedule the next time the nodes start Glossy based on the reference time of the initiator
                rtimer32_set(t_, TIME_16BIT_TO_32BIT(GLOSSY_REFERENCE_TIME + GLOSSY_PERIOD +
                        period_skew - GLOSSY_GUARD_TIME * (1 + sync_missed)), 1,
                        (rtimer_callback_t) fs_scheduler, ptr_);
            }
            // Change state when we are done with syncing
            // We are going to sleep
            last_sender = sender;
            //set_fs_state(FS_STATE_SLEEP);
            set_fs_state(FS_STATE_WAIT);
        } else {
            // Read the ptp header and the sync fields
            fs_header_ptp = (fs_header_ptp_struct *) & fs_header_common->fs_next_field[0];
            fs_read_header_ptp(fs_header_ptp);
            if (fs_header == FS_SETUP_HEADER) {
                sessions_without_sync++;
                destination_id = fs_dest;
                set_dist(fs_src, get_relay_cnt() + 1);
                set_fs_state(FS_STATE_RESPONSE);
            } else {
                // We are intermediate nodes
                // and have missed either the SETUP message or
                // SETUP and RESPONSE messages
                set_fs_state(FS_STATE_DATA);
            }
            if (fs_t_stop == t_heartbeat_start + FS_PERIOD) {
                // We Should check, whether we are here for the first time of a round
                // Calculate the next heartbeat after the SYNC or SETUP message
                t_heartbeat_start = fs_t_stop;
            }
            last_sender = sender;
            rtimer32_set(t_, RTIMER32_TIME(t_) + GLOSSY_PERIOD, 1,
                    (rtimer_callback_t) fs_scheduler, ptr_);
            fs_data = (fs_data_struct *) & fs_header_ptp->fs_next_field[0];
            int ret = fs_read_data(fs_data, packetbuf_dataptr());
            packetbuf_set_datalen(ret);
        }
    } else {
        // Receivers have not received message
        // Schedule the timer
        rtimer32_set(t_, RTIMER32_TIME(t_) + GLOSSY_PERIOD, 1,
                (rtimer_callback_t) fs_scheduler, ptr_);
        if (fs_t_stop == t_heartbeat_start + FS_PERIOD) {
            // We Should check, whether we are here for the first time of a round
            // Calculate the next heartbeat after the SYNC or SETUP message
            t_heartbeat_start = fs_t_stop;
        }
        fs_timeout_cnt++;
        if (fs_timeout_cnt < FS_TIMEOUT_BEFORE_WAIT_STATE) {
            set_fs_state(FS_STATE_SETUP_SYNC);
        } else {
            set_fs_state(FS_STATE_WAIT);
        }
    }
    // Poll the process that prints statistics (will be activated later by Contiki).
    process_poll(&glossy_print_stats_process);
    // Yield the protothread.
    PT_YIELD(pt_);
    PT_END(pt_);
}

PT_THREAD(fs_cb_response_sender(struct pt *pt_, struct rtimer *t_, void *ptr_)) {
    PT_BEGIN(pt_);
    forwarder = 1;
    t_stop = RTIMER32_TIME(t_) + GLOSSY_DURATION;

    fs_write_header_common(fs_header_common, FS_RESPONSE_HEADER, ++fs_seq_no);
    fs_header_ptp = (fs_header_ptp_struct *) & fs_header_common->fs_next_field[0];
    struct dist_info *_dst_dist = lookup_dist_info(fs_src);
    fs_dist = fs_header_ptp->fs_dist_field = (_dst_dist) ? _dst_dist->dist : -1;
    fs_write_header_ptp(fs_header_ptp, node_id, packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[7], fs_dist, fs_slack);
    fs_data = (fs_data_struct *) & fs_header_ptp->fs_next_field[0];
    data_len = FS_HEADER_COMMON_LEN + FS_HEADER_PTP_LEN + FS_DATA_LEN;
    uint8_t app_data_tmp = fs_write_data(fs_data, data_len, 0, packetbuf_dataptr(), packetbuf_datalen());
    data_len += app_data_tmp;
    while (RTIMER_CLOCK_LT(RTIMER32_NOW() - delay, FS_PACKET_CREATION_DELAY_SENDER));
    // Start Glossy.
    glossy_start((uint8_t *) & data_buf[0], &data_len, GLOSSY_INITIATOR, 0, N_TX, t_stop,
            (rtimer_callback_t) fs_scheduler, t_, ptr_);
    // Yield the protothread. It will be resumed when Glossy terminates.
    PT_YIELD(pt_);

    // Stop Glossy.
    glossy_stop();
    if (get_rx_cnt()) {
        fs_timeout_cnt = 0;
        // Read the common header to know what was actually transmitted in the network
        fs_read_header_common(fs_header_common);
        if (fs_header != FS_SYNC_HEADER) {
            fs_header_ptp = (fs_header_ptp_struct *) & fs_header_common->fs_next_field[0];
            fs_read_header_ptp(fs_header_ptp);
            if (fs_header == FS_RESPONSE_HEADER) {
                // remove packet from queue when received application data is the same as sent
                if (fs_src == node_id) {
                    packetqueue_dequeue(&tx_queue); // Remove the item
                    set_dist(fs_src, 0);
                    set_fs_state(FS_STATE_DATA);
                } else {
                    // Someone else was sending a RESPONSE message
                    set_dist(fs_src, get_relay_cnt() + 1);
                    set_fs_state(FS_STATE_RESPONSE);
                    // TODO: Not sure what to do, since it should not happen
                }
            } else {
                if (fs_header == FS_SETUP_HEADER) {
                    set_fs_state(FS_STATE_RESPONSE);
                } else {
                    // Can only be DATA_HEADEr
                    set_fs_state(FS_STATE_DATA);
                }
            }
            // Copy application data and call the Link Layer
            fs_data = (fs_data_struct *) & fs_header_ptp->fs_next_field[0];
            int ret = fs_read_data(fs_data, packetbuf_dataptr());
            packetbuf_set_datalen(ret);
        }
    } else {
        fs_timeout_cnt++;
        if (fs_timeout_cnt < FS_TIMEOUT_SENDER) {
            set_fs_state(FS_STATE_RESPONSE);
        } else {
            set_fs_state(FS_STATE_SLEEP);
        }
    }
    last_sender = sender;
    // Schedule begin of next Glossy phase based on GLOSSY_PERIOD.
    rtimer32_set(t_, RTIMER32_TIME(t_) + GLOSSY_PERIOD, 1, (rtimer_callback_t) fs_scheduler, ptr_);
    // Poll the process that prints statistics (will be activated later by Contiki).
    process_poll(&glossy_print_stats_process);
    // Yield the protothread.
    PT_YIELD(pt_);
    PT_END(pt_);
}

PT_THREAD(fs_cb_response_receiver(struct pt *pt_, struct rtimer *t_, void *ptr_)) {
    PT_BEGIN(pt_);
    forwarder = 1;
    t_stop = RTIMER32_TIME(t_) + GLOSSY_DURATION;
    data_len = 0;
    while (RTIMER_CLOCK_LT(RTIMER32_NOW() - delay, FS_PACKET_CREATION_DELAY_RECEIVER));
    // Start Glossy.
    glossy_start((uint8_t *) & data_buf[0], &data_len, GLOSSY_RECEIVER, 0,
            N_TX, t_stop, (rtimer_callback_t) fs_scheduler, t_, ptr_);
    // Yield the protothread. It will be resumed when Glossy terminates.
    PT_YIELD(pt_);

    // Stop Glossy.
    glossy_stop();
    if (get_rx_cnt()) {
        fs_timeout_cnt = 0;
        fs_read_header_common(fs_header_common);
        if (fs_header != FS_SYNC_HEADER) {
            fs_header_ptp = (fs_header_ptp_struct *) & fs_header_common->fs_next_field[0];
            fs_read_header_ptp(fs_header_ptp);
            if (fs_header == FS_RESPONSE_HEADER) {
                set_dist(fs_src, get_relay_cnt() + 1);
                if (node_id == fs_dest) {
                    // Set the state
                    set_fs_state(FS_STATE_DATA);
                } else {
                    if ((get_dist(fs_src) + get_dist(fs_dest)) <= (fs_dist + fs_slack)) {
                        // We are in the forwarder set
                        // the next sender is the source node
                        // Set the state
                        set_fs_state(FS_STATE_DATA);
                        last_sender = sender;
                    } else {
                        // Calculate random slack
                        if ((get_dist(fs_src) + get_dist(fs_dest)) <= (fs_dist + fs_slack + 1)) {
                            // We would be in the forwarder set when we would have increased the slack by 1
                            // Calculate the chance that we switch to the forwarder set
                            unsigned short rand_value = random_rand();
                            if (rand_value <= (65535 * FS_RANDOM_SLACK) / 100) {
                                last_sender = sender;
                                set_fs_state(FS_STATE_DATA);
                            } else {
                                last_sender = sender;
                                set_fs_state(FS_STATE_SLEEP);
                            }
                        } else {
                            last_sender = sender;
                            set_fs_state(FS_STATE_SLEEP);
                        }
                    }
                }
            } else {
                if (fs_header == FS_SETUP_HEADER) {
                    set_fs_state(FS_STATE_RESPONSE);
                } else {
                    if (fs_header == FS_DATA_HEADER) {
                        // Apparently we missed a response packet
                        // We cannot calculate, if we are in the forwarder set or not
                        // Just in case, we are in the forwarder set
                        // We could also be source
                        if ((fs_src != node_id) && (fs_dest != node_id)) {
                            set_fs_state(FS_STATE_DATA);
                        } else {
                            // We are the source
                            set_fs_state(FS_STATE_RESPONSE);
                        }
                    }
                }
            }
            fs_data = (fs_data_struct *) & fs_header_ptp->fs_next_field[0];
            // Copy application data and call the Link Layer
            int ret = fs_read_data(fs_data, packetbuf_dataptr());
            packetbuf_set_datalen(ret);
            last_sender = sender;
        }
    } else {
        fs_timeout_cnt++;
        if (last_sender) {
            if (fs_timeout_cnt < FS_TIMEOUT_SENDER) {
                set_fs_state(FS_STATE_SETUP_SYNC);
                last_sender = sender;
            } else {
                set_fs_state(FS_STATE_SLEEP);
            }
        } else {
            if (fs_timeout_cnt < FS_TIMEOUT_BEFORE_WAIT_STATE) {
                set_fs_state(FS_STATE_RESPONSE);
            } else {
                set_fs_state(FS_STATE_WAIT);
            }
        }
    }
    // Schedule begin of next Glossy phase based on reference time and GLOSSY_PERIOD.
    rtimer32_set(t_, RTIMER32_TIME(t_) + GLOSSY_PERIOD, 1,
            (rtimer_callback_t) fs_scheduler, ptr_);
    // Poll the process that prints statistics (will be activated later by Contiki).
    process_poll(&glossy_print_stats_process);
    // Yield the protothread.
    PT_YIELD(pt_);
    PT_END(pt_);
}

PT_THREAD(fs_cb_data_sender(struct pt *pt_, struct rtimer *t_, void *ptr_)) {
    PT_BEGIN(pt_);
    forwarder = 1;
    // Glossy phase.
    t_stop = RTIMER32_TIME(t_) + GLOSSY_DURATION;
    fs_write_header_common(fs_header_common, FS_DATA_HEADER, ++fs_seq_no);
    fs_header_ptp = (fs_header_ptp_struct *) & fs_header_common->fs_next_field[0];
    fs_write_header_ptp(fs_header_ptp, node_id, packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[7], fs_dist, fs_slack);
    fs_data = (fs_data_struct *) fs_header_ptp->fs_next_field;
    data_len = FS_HEADER_COMMON_LEN + FS_HEADER_PTP_LEN + FS_DATA_LEN;
    uint8_t app_data_tmp = fs_write_data(fs_data, data_len, 0, packetbuf_dataptr(), packetbuf_datalen());
    data_len += app_data_tmp;
    while (RTIMER_CLOCK_LT(RTIMER32_NOW() - delay, FS_PACKET_CREATION_DELAY_SENDER));
    glossy_start((uint8_t *) & data_buf[0], &data_len, GLOSSY_INITIATOR, 0, N_TX, t_stop,
            (rtimer_callback_t) fs_scheduler, t_, ptr_);
    // Yield the protothread. It will be resumed when Glossy terminates.
    PT_YIELD(pt_);

    // Stop Glossy.
    glossy_stop();
    if (get_rx_cnt()) {
        fs_timeout_cnt = 0;
        // Read the common header to know what was actually transmitted in the network
        fs_read_header_common(fs_header_common);
        if (fs_header != FS_SYNC_HEADER) {
            fs_header_ptp = (fs_header_ptp_struct *) fs_header_common->fs_next_field;
            fs_read_header_ptp(fs_header_ptp);
            if (fs_header == FS_DATA_HEADER) {
                // Check whether we received the same packet as we have sent
                // remove packet from queue when received application data is the same as sent
                if (fs_src == node_id) {
                    packetqueue_dequeue(&tx_queue); // Remove the item
                } else {
                    // We got a packet from someone else, but we are sender
                    // Pass the data to application layer and
                    // flip a coin whether we are sender or in the next round
                    // Something like random backoff in other mac protocols
                    flip_a_coin = 1;
                }
                set_fs_state(FS_STATE_DATA);
            } else {
                if (fs_header == FS_RESPONSE_HEADER) {
                    set_fs_state(FS_STATE_DATA);
                } else {
                    set_fs_state(FS_STATE_DATA);
                }
            }
            fs_data = (fs_data_struct *) & fs_header_ptp->fs_next_field[0];
            int ret = fs_read_data(fs_data, packetbuf_dataptr());
            packetbuf_set_datalen(ret);
            last_sender = sender;
        }
    } else {
        fs_timeout_cnt++;
        if (fs_timeout_cnt < FS_TIMEOUT_SENDER) {
            set_fs_state(FS_STATE_DATA);
            last_sender = sender;
        } else {
            set_fs_state(FS_STATE_SLEEP);
            last_sender = sender;
        }
    }
    // Schedule begin of next Glossy phase based on GLOSSY_PERIOD.
    rtimer32_set(t_, RTIMER32_TIME(t_) + GLOSSY_PERIOD, 1, (rtimer_callback_t) fs_scheduler, ptr_);
    // Poll the process that prints statistics (will be activated later by Contiki).
    process_poll(&glossy_print_stats_process);
    // Yield the protothread.
    PT_YIELD(pt_);
    PT_END(pt_);
}

PT_THREAD(fs_cb_data_receiver(struct pt *pt_, struct rtimer *t_, void *ptr_)) {
    PT_BEGIN(pt_);
    forwarder = 1;
    t_stop = RTIMER32_TIME(t_) + GLOSSY_DURATION;
    data_len = 0;
    while (RTIMER_CLOCK_LT(RTIMER32_NOW() - delay, FS_PACKET_CREATION_DELAY_RECEIVER));
    // Start Glossy.
    glossy_start((uint8_t *) & data_buf[0], &data_len, GLOSSY_RECEIVER, 0,
            N_TX, t_stop, (rtimer_callback_t) fs_scheduler, t_, ptr_);
    // Yield the protothread. It will be resumed when Glossy terminates.
    PT_YIELD(pt_);

    // Stop Glossy.
    glossy_stop();
    if (get_rx_cnt()) {
        fs_timeout_cnt = 0;
        fs_read_header_common(fs_header_common);
        if (fs_header != FS_SYNC_HEADER) {
            fs_header_ptp = (fs_header_ptp_struct *) fs_header_common->fs_next_field;
            fs_read_header_ptp(fs_header_ptp);
            if (fs_header == FS_DATA_HEADER) {
                set_fs_state(FS_STATE_DATA);
            } else {
                if (fs_header == FS_RESPONSE_HEADER) {
                    set_fs_state(FS_STATE_DATA);
                } else {
                    if (fs_header == FS_SETUP_HEADER) {
                        // Set distance: source - node
                        set_dist(fs_src, get_relay_cnt() + 1);
                        set_fs_state(FS_STATE_RESPONSE);
                    } else {
                        set_fs_state(FS_STATE_DATA);
                    }
                }
            }
            fs_data = (fs_data_struct *) & fs_header_ptp->fs_next_field[0];
            int ret = fs_read_data(fs_data, packetbuf_dataptr());
            packetbuf_set_datalen(ret);
            last_sender = sender;
        }
    } else {
        fs_timeout_cnt++;
        // We await application data
        // but maybe the source has missed the RESPONSE message
        if (node_id == destination_id) {
            // FIXME We might have a deadlock here
            set_fs_state(FS_STATE_RESPONSE);
            // Test: increase timeout again
        } else {
            if (fs_timeout_cnt < FS_TIMEOUT_BEFORE_WAIT_STATE) {
                set_fs_state(FS_STATE_DATA);
            } else {
                set_fs_state(FS_STATE_WAIT);
            }
        }
    }
    // Schedule begin of next Glossy phase based on reference time and GLOSSY_PERIOD.
    rtimer32_set(t_, RTIMER32_TIME(t_) + GLOSSY_PERIOD, 1,
            (rtimer_callback_t) fs_scheduler, ptr_);
    // Poll the process that prints statistics (will be activated later by Contiki).
    process_poll(&glossy_print_stats_process);
    // Yield the protothread.
    PT_YIELD(pt_);
    PT_END(pt_);
}

PT_THREAD(fs_cb_wait(struct pt *pt_, struct rtimer *t_, void *ptr_)) {
    PT_BEGIN(pt_);
    forwarder = 1;
    t_stop = RTIMER32_TIME(t_) + GLOSSY_DURATION;
    data_len = 0;
    while (RTIMER_CLOCK_LT(RTIMER32_NOW() - delay, FS_PACKET_CREATION_DELAY_RECEIVER));
    // Start Glossy.
    glossy_start((uint8_t *) & data_buf[0], &data_len, GLOSSY_RECEIVER, 0,
            N_TX, t_stop, (rtimer_callback_t) fs_scheduler, t_, ptr_);
    // Yield the protothread. It will be resumed when Glossy terminates.
    PT_YIELD(pt_);

    // Stop Glossy.
    glossy_stop();
    if (get_rx_cnt()) {
        fs_timeout_cnt = 0;
        fs_read_header_common(fs_header_common);
        if (fs_header != FS_SYNC_HEADER) {
            fs_header_ptp = (fs_header_ptp_struct *) fs_header_common->fs_next_field;
            fs_read_header_ptp(fs_header_ptp);
            if (fs_header == FS_DATA_HEADER) {
                // Data received
                set_fs_state(FS_STATE_DATA);
            } else {
                if (fs_header == FS_SETUP_HEADER) {
                    destination_id = fs_dest;
                    set_dist(fs_src, get_relay_cnt() + 1);
                    set_fs_state(FS_STATE_RESPONSE);
                } else {
                    if (fs_header == FS_RESPONSE_HEADER) {
                        //set_fs_state(FS_STATE_DATA);
                        set_dist(fs_src, get_relay_cnt() + 1);
                        if (node_id == fs_dest) {
                            // Set the state
                            set_fs_state(FS_STATE_DATA);
                        } else {
                            if ((get_dist(fs_src) + get_dist(fs_dest)) <= (fs_dist + fs_slack)) {
                                // We are in the forwarder set
                                // the next sender is the source node
                                // Set the state
                                set_fs_state(FS_STATE_DATA);
                                last_sender = sender;
                            } else {
                                // Calculate random slack
                                if ((get_dist(fs_src) + get_dist(fs_dest)) <= (fs_dist + fs_slack + 1)) {
                                    // We would be in the forwarder set when we would have increased the slack by 1
                                    // Calculate the chance that we switch to the forwarder set
                                    unsigned short rand_value = random_rand();
                                    if (rand_value <= (65535 * FS_RANDOM_SLACK) / 100) {
                                        last_sender = sender;
                                        set_fs_state(FS_STATE_DATA);
                                    } else {
                                        last_sender = sender;
                                        set_fs_state(FS_STATE_SLEEP);
                                    }
                                } else {
                                    last_sender = sender;
                                    set_fs_state(FS_STATE_SLEEP);
                                }
                            }
                        }

                    }
                }
            }
        }
        // Store source, destination and slack value from the header
        fs_data = (fs_data_struct *) & fs_header_ptp->fs_next_field[0];
        int ret = fs_read_data(fs_data, packetbuf_dataptr());
        packetbuf_set_datalen(ret);
        last_sender = sender;
    } else {
        // We have not received a message, so we can go to sleep
        set_fs_state(FS_STATE_SLEEP);
    }
    // Schedule begin of next Glossy phase based on reference time and GLOSSY_PERIOD.
    rtimer32_set(t_, RTIMER32_TIME(t_) + GLOSSY_PERIOD, 1,
            (rtimer_callback_t) fs_scheduler, ptr_);
    // Poll the process that prints statistics (will be activated later by Contiki).
    process_poll(&glossy_print_stats_process);
    // Yield the protothread.
    PT_YIELD(pt_);
    PT_END(pt_);
}

PT_THREAD(fs_cb_sleep(struct pt *pt_, struct rtimer *t_, void *ptr_)) {
    PT_BEGIN(pt_);
    forwarder = 0;
    fs_timeout_cnt = 0;
    // Just for debugging
    set_fs_state(FS_STATE_SLEEP);
    // Schedule timer so we can check the time
    rtimer32_set(t_, RTIMER32_TIME(t_) + GLOSSY_PERIOD, 1, (rtimer_callback_t) fs_scheduler, ptr_);
    // Poll the process that prints statistics (will be activated later by Contiki).
    process_poll(&glossy_print_stats_process);
    PT_YIELD(pt_);
    PT_END(pt_);
}

char fs_scheduler(struct rtimer *t, void *ptr) {
    PT_BEGIN(&fs_pt);

    while (1) {
        // Schedule heartbeat
        fs_t_stop = t_heartbeat_start + FS_PERIOD;
        // initiator (aka border router) starts the round
        last_sender = sender;
        sender = (node_id == INITIATOR_NODE_ID);
        // Empty the packet buffer
        memset(data_buf, 0, sizeof (data_buf));
        fs_flags = 0;
        sync_flag = 1;
        forwarder = 0;
        fs_timeout_cnt = 0;
        fs_src = 0;
        fs_dest = 0;
        send_data = 0;

        // At the beginning of the heartbeat, the initiator always starts sending a message.
        set_fs_state(FS_STATE_SETUP_SYNC);
        // Wait in case the current heartbeat has not started yet
        if (t_heartbeat_start > RTIMER32_NOW()) {
            rtimer32_set(t, t_heartbeat_start, 1, (rtimer_callback_t) fs_scheduler, ptr);
            PT_YIELD(&fs_pt);
        }

        while (RTIMER32_CLOCK_LT(RTIMER32_NOW() + GLOSSY_PERIOD + FS_SESSION_GUARD, fs_t_stop)) {
            delay = RTIMER32_NOW();
            static struct pt fs_child_pt;
            if (RTIMER32_CLOCK_LT(RTIMER32_NOW() + FS_SESSION_GUARD + (2 * GLOSSY_PERIOD), fs_t_stop)) {
                if (((fs_state == FS_STATE_SETUP_SYNC) && (node_id == INITIATOR_NODE_ID)) ||
                        (fs_state == FS_STATE_RESPONSE && node_id == destination_id) ||
                        (fs_state == FS_STATE_DATA) ||
                        (fs_state == FS_STATE_WAIT)) {
                    // Check if we have application data
                    if (packetqueue_len(&tx_queue) > 0) {
                        laneflood_packetqueue_item = packetqueue_first(&tx_queue);
                        if (laneflood_packetqueue_item != NULL) {
                            laneflood_queuebuf_item = packetqueue_queuebuf(laneflood_packetqueue_item);
                            if (laneflood_queuebuf_item != NULL) {
                                packetbuf_clear();
                                queuebuf_to_packetbuf(laneflood_queuebuf_item);
                                if (node_id == INITIATOR_NODE_ID) {
                                    send_data = 1;
                                }
                                sender = 1;
                            }
                        }
                    }
                }
            }
            if (sender && fs_state == FS_STATE_WAIT && node_id != INITIATOR_NODE_ID) {
                // initiator is ready?
                fs_state = FS_STATE_SETUP_SYNC;
            } else {
                if (flip_a_coin && sender) {
                    flip_a_coin = 0;
                    if (random_rand() % 2) {
                        // if 1 you can send
                        sender = 1;
                    } else {
                        sender = 0;
                    }
                }
            }
            PT_SPAWN(&fs_pt, &fs_child_pt, laneflood_table[sender][fs_state](&fs_child_pt, t, NULL));
            // Reset the sender
            sender = 0;
        }
    }
    PT_END(&fs_pt);
}

char glossy_scheduler(struct rtimer *t, void *ptr) {
    // This is for Bootstrapping
    PT_BEGIN(&pt);
    // Set LaneFlood state to OFF
    set_fs_state(FS_STATE_OFF);

    while (fs_state == FS_STATE_OFF) {
        if (IS_INITIATOR()) { // Glossy initiator.
            rtimer32_clock_t t_stop;
            if (GLOSSY_IS_BOOTSTRAPPING()) {
                // Nodes need to be synchronised and bootstrapped,
                // thus Glossy must have been running at least GLOSSY_BOOTSTRAP_PERIODS
                fs_write_header_common(fs_header_common, GLOSSY_HEADER, ++fs_seq_no);
            } else {
                // We are done with the initial boostrapping
                // Set the the FS_SYNC_HEADER so the nodes can learn and calculate the first heartbeat
                fs_write_header_common(fs_header_common, FS_SYNC_HEADER, ++fs_seq_no);
                fs_sync = (fs_sync_struct *) fs_header_common->fs_next_field;
                fs_sync->fs_seq_no_to_sync = uip_htons(seq_no_to_sync);
            }
            // Schedule end of Glossy phase based on GLOSSY_DURATION.
            t_stop = RTIMER32_TIME(t) + GLOSSY_DURATION;
            data_len = FS_HEADER_COMMON_LEN + FS_SYNC_LEN;
            // Glossy phase.
            glossy_start((uint8_t *) & data_buf[0], &data_len, GLOSSY_INITIATOR, 1, N_TX, t_stop,
                    (rtimer_callback_t) glossy_scheduler, t, ptr);
            // Store time at which Glossy has started.
            t_start = RTIMER32_TIME(t);
            // Yield the protothread. It will be resumed when Glossy terminates.
            PT_YIELD(&pt);

            // Stop Glossy.
            glossy_stop();
            // Read the common header
            fs_read_header_common(fs_header_common);
            // Increase the current sequence number otherwise the initiator schedules one phase too much
            cur_seq_no = fs_seq_no + 1;
            if (!GLOSSY_IS_BOOTSTRAPPING()) {
                // Glossy has already successfully bootstrapped.
                if (!GLOSSY_IS_SYNCED()) {
                    // The reference time was not updated: increment reference time by GLOSSY_PERIOD.
                    set_t_ref_l(GLOSSY_REFERENCE_TIME + GLOSSY_PERIOD);
                    set_t_ref_l_updated(1);
                }
            }
            if (GLOSSY_IS_BOOTSTRAPPING() || FS_IS_BOOTSTRAPPING()) {
                // Schedule begin of next Glossy phase based on GLOSSY_PERIOD.
                rtimer32_set(t, t_start + GLOSSY_PERIOD, 1, (rtimer_callback_t) glossy_scheduler, ptr);
                // Estimate the clock skew over the last period.
                estimate_period_skew();
                // Set last_sender for statistics
                last_sender = 1;
                process_poll(&glossy_print_stats_process);
                // Reset the data buffer
                memset(data_buf, 0, sizeof (data_buf));
                // Yield the protothread.
                PT_YIELD(&pt);
            } else {
                // It's time to schedule the heartbeat
                t_heartbeat_start = t_start + GLOSSY_PERIOD;
                // Schedule the heartbeat
                rtimer32_set(t, t_heartbeat_start - GLOSSY_DURATION, 1, (rtimer_callback_t) fs_scheduler, NULL);
                // Estimate the clock skew over the last period.
                estimate_period_skew();
                // Set last_sender for statistics
                last_sender = 1;
                // Set the forwarder selection state
                set_fs_state(FS_STATE_SETUP_SYNC);
                energest_init();
                // Poll the process that prints statistics (will be activated later by Contiki).
                process_poll(&glossy_print_stats_process);
            }
        } else { // Glossy receiver.
            // Glossy phase.
            rtimer32_clock_t t_stop;
            if (GLOSSY_IS_BOOTSTRAPPING()) {
                // Glossy is still bootstrapping:
                // Schedule end of Glossy phase based on GLOSSY_INIT_DURATION.
                t_stop = RTIMER32_TIME(t) + GLOSSY_INIT_DURATION;
            } else {
                // Glossy has already successfully bootstrapped:
                // Schedule end of Glossy phase based on GLOSSY_DURATION.
                t_stop = RTIMER32_TIME(t) + GLOSSY_DURATION;
            }
            data_len = 0;
            // Start Glossy.
            glossy_start((uint8_t *) & data_buf[0], &data_len, GLOSSY_RECEIVER, 1,
                    N_TX, t_stop, (rtimer_callback_t) glossy_scheduler, t, ptr);
            // Yield the protothread. It will be resumed when Glossy terminates.
            PT_YIELD(&pt);

            // Stop Glossy.
            glossy_stop();
            if (get_rx_cnt()) {
                // Read the common header
                fs_read_header_common(fs_header_common);
                if (!GLOSSY_IS_BOOTSTRAPPING()) {
                    cur_seq_no = fs_seq_no;
                }
            }
            if (GLOSSY_IS_BOOTSTRAPPING()) {
                // Glossy is still bootstrapping.
                if (!GLOSSY_IS_SYNCED()) {
                    // The reference time was not updated: reset skew_estimated to zero.
                    skew_estimated = 0;
                }
            } else {
                // Glossy has already successfully bootstrapped.
                if (!GLOSSY_IS_SYNCED()) {
                    // The reference time was not updated:
                    // increment reference time by GLOSSY_PERIOD + period_skew.
                    set_t_ref_l(GLOSSY_REFERENCE_TIME + GLOSSY_PERIOD + period_skew);
                    set_t_ref_l_updated(1);
                    // Increment sync_missed.
                    sync_missed++;
                } else {
                    // The reference time not updated: reset sync_missed to zero.
                    sync_missed = 0;
                }
            }
            // Estimate the clock skew over the last period.
            estimate_period_skew();
            if (get_rx_cnt()) {
                fs_sync = (fs_sync_struct *) fs_header_common->fs_next_field;
                fs_read_sync(fs_sync);
            }
            if (!GLOSSY_IS_BOOTSTRAPPING() && !seq_no_to_sync) {
                // Reference sequence number not set, yet.
                // We do not wait explicitly for a message from the receiver.
                // In case a node does not receive anything,
                // it is still able to calculate the first heartbeat
                if (!get_rx_cnt()) {
                    // We have not received a message
                    // Just in case, schedule the heartbeat
                    t_heartbeat_start = TIME_16BIT_TO_32BIT(GLOSSY_REFERENCE_TIME + (FS_INIT_SYNC_GUARD + 1) * GLOSSY_PERIOD);
                }
            }
            // TODO: This is ugly
            uint8_t msg_to_receive = 10;
            if (get_rx_cnt() && fs_header == FS_SYNC_HEADER) {
                // SYNC msg received, calulate/update the start of the heartbeat
                // Calulcate the amount of message exchanges left
                msg_to_receive = seq_no_to_sync - uip_htons(fs_header_common->fs_seq_no);
                if (msg_to_receive) {
                    t_heartbeat_start = TIME_16BIT_TO_32BIT(GLOSSY_REFERENCE_TIME + (msg_to_receive + 1) * GLOSSY_PERIOD);
                } else {
                    t_heartbeat_start = TIME_16BIT_TO_32BIT(GLOSSY_REFERENCE_TIME + GLOSSY_PERIOD + period_skew - GLOSSY_GUARD_TIME * (1 + sync_missed));
                }
            }
            if ((t_heartbeat_start == 0) || (msg_to_receive)) {
                // t_heartbeat_start is not set now or
                // we can savely schedule at least one more Glossy wave
                if (GLOSSY_IS_BOOTSTRAPPING()) {
                    // Glossy is still bootstrapping.
                    if (skew_estimated == 0) {
                        // The reference time was not updated:
                        // Schedule begin of next Glossy phase based on last begin and GLOSSY_INIT_PERIOD.
                        rtimer32_set(t, RTIMER32_TIME(t) + GLOSSY_INIT_PERIOD, 1,
                                (rtimer_callback_t) glossy_scheduler, ptr);
                    } else {
                        // The reference time was updated:
                        // Schedule begin of next Glossy phase based on reference time and GLOSSY_INIT_PERIOD.
                        rtimer32_set(t, TIME_16BIT_TO_32BIT(GLOSSY_REFERENCE_TIME + GLOSSY_PERIOD - GLOSSY_INIT_GUARD_TIME),
                                1, (rtimer_callback_t) glossy_scheduler, ptr);
                    }
                } else {
                    // Glossy has already successfully bootstrapped:
                    // Schedule begin of next Glossy phase based on reference time and GLOSSY_PERIOD.
                    rtimer32_set(t, TIME_16BIT_TO_32BIT(GLOSSY_REFERENCE_TIME + GLOSSY_PERIOD + period_skew - GLOSSY_GUARD_TIME * (1 + sync_missed)),
                            1, (rtimer_callback_t) glossy_scheduler, ptr);
                }
                // Poll the process that prints statistics (will be activated later by Contiki).
                process_poll(&glossy_print_stats_process);
                // Reset the data buffer
                memset(data_buf, 0, sizeof (data_buf));
                // Yield the protothread.
                PT_YIELD(&pt);
            } else {
                // We are close to the heartbeat, lets schedule it
                rtimer32_set(t, t_heartbeat_start - GLOSSY_DURATION, 1, (rtimer_callback_t) fs_scheduler, NULL);
                // Set last_sender for statistics
                last_sender = 0;
                // Set the forwarder selection state
                set_fs_state(FS_STATE_SETUP_SYNC);
                energest_init();
                // Poll the process that prints statistics (will be activated later by Contiki).
                process_poll(&glossy_print_stats_process);
            }
        }
    }
    PT_END(&pt);
}

PROCESS(laneflood_process, "LaneFlood Process");

PROCESS_THREAD(laneflood_process, ev, data) {
    PROCESS_BEGIN();

    printf("LaneFlood process started \n");

    // Initialise the common header
    fs_header_common = (fs_header_common_struct *) & data_buf[0];
    // Reset the data buffer
    memset(data_buf, 0, sizeof (data_buf));
    // Initialize the sequence number.
    fs_seq_no = 0;
    // Start print stats processes.
    process_start(&glossy_print_stats_process, NULL);
    // Start Glossy busy-waiting process.
    process_start(&glossy_process, NULL);
    // Wait for some time to ensure that all nodes are awake
    if (IS_INITIATOR()) {
        // Initiator waits for all other nodes to boot up.
#if COOJA || LOCAL_NODES || INDRIYA
        etimer_set(&et, 2 * CLOCK_SECOND);
#else
        etimer_set(&et, 30 * CLOCK_SECOND);
#endif /* COOJA */
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    }
#if GLOSSY_DEBUG_STATUS
    // Initialise the debug status buffer
    memset(debug_status, DEBUG_WAITING, sizeof (debug_status));
#endif /* GLOSSY_DEBUG_STATUS */
    // Set the upper 16 bit of the virtual 32 bit timer
    rtimer32_arch_handle_TAR_overflow();
    // Start Glossy experiment in one second.
    rtimer32_set(&rt, RTIMER32_NOW() + (RTIMER_SECOND), 1, (rtimer_callback_t) glossy_scheduler, NULL);

    PROCESS_END();
}
