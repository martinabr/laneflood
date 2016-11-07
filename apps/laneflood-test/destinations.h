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

#ifndef APPS_GLOSSY_TEST_DESTINATIONS_H_
#define APPS_GLOSSY_TEST_DESTINATIONS_H_

#if COOJA
uint16_t server_addr[3][8] = {
		{0xaaaa, 0x0, 0x0, 0x0, 0x212, 0x7402, 0x2, 0x2}, // node id 2
		{0xaaaa, 0x0, 0x0, 0x0, 0x212, 0x7404, 0x4, 0x4}, // node id 4
		{0xaaaa, 0x0, 0x0, 0x0, 0x212, 0x7407, 0x7, 0x7} // node id 7
};
#elif FLOCKLAB
uint16_t server_addr[3][8] = {
		{0xaaaa, 0x0, 0x0, 0x0, 0x212, 0x7400, 0x10d0, 0x10}, // node id 16
		{0xaaaa, 0x0, 0x0, 0x0, 0x212, 0x7400, 0xeda, 0xd}, // node id 13
		{0xaaaa, 0x0, 0x0, 0x0, 0x212, 0x7400, 0x115e, 0x7} // node id 7
};
#elif INDRIYA
uint16_t server_addr[3][8] = {
		{0xaaaa, 0x0, 0x0, 0x0, 0x212, 0x7400, 0x146e, 0x2}, // node id 2
		{0xaaaa, 0x0, 0x0, 0x0, 0x212, 0x7400, 0xed5, 0x46}, // node id 70
		{0xaaaa, 0x0, 0x0, 0x0, 0x212, 0x7400, 0x12e5, 0x79} // node id 121
};
#endif /* COOJA */

#endif /* APPS_GLOSSY_TEST_DESTINATIONS_H_ */
