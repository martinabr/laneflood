/*
 * Copyright (c) 2012, Thingsquare, http://www.thingsquare.com/.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef CONTIKI_DEFAULT_CONF_H
#define CONTIKI_DEFAULT_CONF_H

/*---------------------------------------------------------------------------*/
/* Packet buffer size options.
 *
 * The packet buffer size options can be tweaked on a per-project
 * basis to reduce memory consumption.
 */

/* QUEUEBUF_CONF_NUM specifies the number of queue buffers. Queue
   buffers are used throughout the Contiki netstack but the
   configuration option can be tweaked to save memory. Performance can
   suffer with a too low number of queue buffers though. */
#ifndef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM 8
#endif /* QUEUEBUF_CONF_NUM */
/*---------------------------------------------------------------------------*/
/* uIPv6 configuration options.
 *
 * Many of the uIPv6 configuration options can be overriden by a
 * project-specific configuration to save memory.
 */

/* NETSTACK_CONF_WITH_IPV6 specifies whether or not IPv6 should be used. If IPv6
   is not used, IPv4 is used instead. */
#ifndef NETSTACK_CONF_WITH_IPV6
#define NETSTACK_CONF_WITH_IPV6 0
#endif /* NETSTACK_CONF_WITH_IPV6 */

/* UIP_CONF_BUFFER_SIZE specifies how much memory should be reserved
   for the uIP packet buffer. This sets an upper bound on the largest
   IP packet that can be received by the system. */
#ifndef UIP_CONF_BUFFER_SIZE
#define UIP_CONF_BUFFER_SIZE 128
#endif /* UIP_CONF_BUFFER_SIZE */

/* UIP_CONF_UDP specifies if UDP support should be included or
   not. Disabling UDP saves memory but breaks a lot of stuff. */
#ifndef UIP_CONF_UDP
#define UIP_CONF_UDP 1
#endif /* UIP_CONF_UDP */

/* UIP_CONF_MAX_CONNECTIONS specifies the maximum number of
   simultaneous TCP connections. */
#ifndef UIP_CONF_MAX_CONNECTIONS
#define UIP_CONF_MAX_CONNECTIONS 8
#endif /* UIP_CONF_MAX_CONNECTIONS */

/* UIP_CONF_TCP specifies if TCP support should be included or
   not. Disabling TCP saves memory. */
#ifndef UIP_CONF_TCP
#define UIP_CONF_TCP 1
#endif /* UIP_CONF_TCP */

/* UIP_CONF_MAX_CONNECTIONS specifies the maximum number of
   simultaneous TCP connections. */
#ifndef UIP_CONF_MAX_CONNECTIONS
#define UIP_CONF_MAX_CONNECTIONS 8
#endif /* UIP_CONF_MAX_CONNECTIONS */


#endif /* CONTIKI_DEFAULT_CONF_H */
