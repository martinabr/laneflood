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

#ifndef CORE_NET_MAC_LANEFLOOD_LANEFLOOD_H_
#define CORE_NET_MAC_LANEFLOOD_LANEFLOOD_H_

#include "../laneflood/laneflood_interface.h"
#include "contiki.h"

#include "glossy.h"
#include "node-id.h"
#include "uthash/utlist.h"
#include "lib/memb.h"
#include "lib/random.h"

typedef uint8_t NODE_ID_TYPE;

extern struct packetqueue tx_queue;

/**
 * \brief NodeId of the initiator.
 *        Default value: 1
 */
#ifndef INITIATOR_NODE_ID
#define INITIATOR_NODE_ID       1
#endif

#define N_TX	5

#define MAX_DISTANCES_COUNT 10

/**
 * \brief Period with which a Glossy phase is scheduled.
 *        Default value: 250 ms.
 */
#define GLOSSY_PERIOD          (RTIMER_SECOND / 5) // 200 ms

#define FS_SESSION_GUARD		(GLOSSY_PERIOD / 2)


/**
 * \brief Duration of each Glossy phase.
 *        Default value: 20 ms.
 */
#define GLOSSY_DURATION         (RTIMER_SECOND / 15) // 67 ms

/**
 * \brief Guard-time at receivers.
 *        Default value: 526 us.
 */
#if COOJA
#define GLOSSY_GUARD_TIME       (RTIMER_SECOND / 800)
#else
#define GLOSSY_GUARD_TIME       (RTIMER_SECOND / 1900)   // 526 us
#endif /* COOJA */

/**
 * \brief Period during bootstrapping at receivers.
 *        It should not be an exact fraction of \link GLOSSY_PERIOD \endlink.
 *        Default value: 69.474 ms.
 */
#define GLOSSY_INIT_PERIOD      (GLOSSY_INIT_DURATION + RTIMER_SECOND / 100)                   //  69.474 ms

/**
 * \brief Duration during bootstrapping at receivers.
 *        Default value: 59.474 ms.
 */
#define GLOSSY_INIT_DURATION    (GLOSSY_DURATION - GLOSSY_GUARD_TIME + GLOSSY_INIT_GUARD_TIME) //  59.474 ms

/**
 * \brief Guard-time during bootstrapping at receivers.
 *        Default value: 50 ms.
 */
#define GLOSSY_INIT_GUARD_TIME  (RTIMER_SECOND / 20)                                           //  50 ms

/**
 * Duration of the heartbeat
 */
#define FS_PERIOD				(RTIMER_SECOND * 4UL)

#define FS_INIT_SYNC_GUARD		5

#define FS_SYNC_GUARD			1 // TODO: Currently does *not* work with other values

#define FS_FLAG_ACK	0x01

#define FS_INIT_SLACK	27
#define FS_RANDOM_SLACK 00  // Value in %; can be between 0 and 99
                            // in Flocklab with signal strength -10dBm, the slack should be 0.80

#define FS_TIMEOUT_BEFORE_WAIT_STATE 	2
#define FS_TIMEOUT_SENDER 			(FS_TIMEOUT_BEFORE_WAIT_STATE)

#define FS_PACKET_CREATION_DELAY_SENDER 	41
#define FS_PACKET_CREATION_DELAY_RECEIVER 	41

/**
 * \brief Number of consecutive Glossy phases with successful computation of reference time required to exit from bootstrapping.
 *        Default value: 3.
 */
#define GLOSSY_BOOTSTRAP_PERIODS 5
/**
 * \brief Check if Glossy is still bootstrapping.
 * \sa \link GLOSSY_BOOTSTRAP_PERIODS \endlink.
 */
#define GLOSSY_IS_BOOTSTRAPPING()   (skew_estimated < GLOSSY_BOOTSTRAP_PERIODS)

enum fs_header_type {
	GLOSSY_HEADER,
	FS_SYNC_HEADER,
	FS_SETUP_HEADER,
	FS_RESPONSE_HEADER,
	FS_DATA_HEADER
};
/**
 * List of possible FS states
 */
enum forwarder_selection_state {
 	FS_STATE_SETUP_SYNC,
 	FS_STATE_RESPONSE,
 	FS_STATE_DATA,
	FS_STATE_WAIT,
 	FS_STATE_SLEEP,
 	FS_STATE_OFF
 };

/**
 * \brief	The common header struct
 *
 * 			Header struct that is used by all packets
 */
typedef struct {
	uint8_t fs_header_field;
	uint16_t fs_seq_no;
	uint8_t fs_next_field[];
} __attribute__((packed)) fs_header_common_struct;
/**
 * \brief	The FS header struct
 *
 * 			Header struct that is used for point-to-point communication.
 */
typedef struct {
	NODE_ID_TYPE fs_src_field;
	NODE_ID_TYPE fs_dest_field;
	uint8_t	fs_dist_field;
	uint8_t fs_slack_field;
	uint8_t	fs_next_field[];
} __attribute__((packed)) fs_header_ptp_struct;
/**
 * \brief	FS data structure used to represent data.
 *
 * 			Data struct that is used to represent data during point-to-point communication.
 */
typedef struct {
	uint8_t  fs_data_flags;
	uint8_t  fs_payload[];			/**< Application payload. */
} __attribute__((packed)) fs_data_struct;

/**
 * \brief	FS data structure to sync to the common heartbeat
 *
 * 			Data struct that is used to represent data during point-to-point communication.
 */
typedef struct {
	uint16_t  fs_seq_no_to_sync;
	uint8_t	  fs_next_field[];
} __attribute__((packed)) fs_sync_struct;

void set_fs_state(uint8_t state);

/**
 * \brief Length of data structure.
 */
#define FS_HEADER_COMMON_LEN		sizeof(fs_header_common_struct)
#define FS_HEADER_PTP_LEN			sizeof(fs_header_ptp_struct)
#define FS_DATA_LEN					sizeof(fs_data_struct)
#define FS_GLOSSY_HEADER_LEN		(sizeof(fs_header_glossy_struct) + FOOTER_LEN)
#define FS_SYNC_LEN					sizeof(fs_sync_struct)

/**
 * \brief Check if the nodeId matches the one of the initiator.
 */
#define IS_INITIATOR()              (node_id == INITIATOR_NODE_ID)



#define FS_IS_BOOTSTRAPPING()		((signed short)(seq_no_to_sync - cur_seq_no) >= 0)

/**
 * \brief Check if Glossy is synchronized.
 *
 * The application assumes that a node is synchronized if it updated the reference time
 * during the last Glossy phase.
 * \sa \link is_t_ref_l_updated \endlink
 */
#define GLOSSY_IS_SYNCED()          (is_t_ref_l_updated())

/**
 * \brief Get Glossy reference time.
 * \sa \link get_t_ref_l \endlink
 */
#define GLOSSY_REFERENCE_TIME       (get_t_ref_l())

char glossy_scheduler(struct rtimer *t, void *ptr);

char fs_scheduler(struct rtimer *t, void *ptr);


/** Number of states */
#define NUM_STATES	5
/** Number of possible roles */
#define NUM_ROLES 2

PT_THREAD(fs_cb_setup_sync_sender(struct pt *pt_, struct rtimer *t_, void *ptr_));
PT_THREAD(fs_cb_setup_sync_receiver(struct pt *pt_, struct rtimer *t_, void *ptr_));
PT_THREAD(fs_cb_response_sender(struct pt *pt_, struct rtimer *t_, void *ptr_));
PT_THREAD(fs_cb_response_receiver(struct pt *pt_, struct rtimer *t_, void *ptr_));
PT_THREAD(fs_cb_data_sender(struct pt *pt_, struct rtimer *t_, void *ptr_));
PT_THREAD(fs_cb_data_receiver(struct pt *pt_, struct rtimer *t_, void *ptr_));
PT_THREAD(fs_cb_wait(struct pt *pt_, struct rtimer *t_, void *ptr_));
PT_THREAD(fs_cb_sleep(struct pt *pt_, struct rtimer *t_, void *ptr_));


/** General callback function definition, it corresponds to a function
 * to be called in a position of the state machine table */
typedef char (*sm_action)();



typedef struct dist_info {
	NODE_ID_TYPE id;
	uint8_t dist;
	struct dist_info* next;
	struct dist_info* prev;
} dist_info;
struct dist_info* get_dist_head();

// TODO: comment
void set_dist(NODE_ID_TYPE src, uint8_t distance);
void shrink_dists();
uint8_t get_tx_cnt(void);
struct dist_info* lookup_dist_info(NODE_ID_TYPE node);

#if GLOSSY_DEBUG
uint16_t bad_header;
#endif /* GLOSSY_DEBUG */

PROCESS_NAME(laneflood_process);

#endif /* CORE_NET_MAC_LANEFLOOD_LANEFLOOD_H_ */
