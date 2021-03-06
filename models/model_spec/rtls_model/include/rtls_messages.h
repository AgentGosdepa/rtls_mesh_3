/* Copyright (c) 2010 - 2020, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RTLS_MESSAGES_H__
#define RTLS_MESSAGES_H__

#include <stdint.h>

#define RTLS_PULSE_SET_LEN 2
#define RTLS_PRESSURE_SET_LEN 3
#define RTLS_RSSI_SET_LEN 4

typedef enum
{
    RTLS_OPCODE_PULSE_SET = 0x8202,
    RTLS_OPCODE_PULSE_SET_UNACKNOWLEDGED = 0x8203,
    RTLS_OPCODE_PRESSURE_SET = 0x8201,
    RTLS_OPCODE_PRESSURE_SET_UNACKNOWLEDGED = 0x8205,
    RTLS_OPCODE_RSSI_SET = 0x8206,
    RTLS_OPCODE_RSSI_SET_UNACKNOWLEDGED = 0x8207,
    RTLS_OPCODE_STATUS = 0x8204
} rtls_opcode_t;

typedef struct __attribute((packed))
{
	uint8_t tid;
	union __attribute((packed))
	{
		uint8_t pulse;

		struct __attribute((packed))
		{
			uint8_t pressure_up;
			uint8_t pressure_down;
		} pressure;

		struct __attribute((packed))
		{
			uint16_t tag_id;
			uint8_t rssi;
		} rssi;
	};
} rtls_set_msg_pkt_t;

typedef union __attribute((packed))
{
	uint8_t tid;
	union __attribute((packed))
	{
		uint8_t pulse;
		struct __attribute((packed))
		{
			uint8_t pressure_up;
			uint8_t pressure_down;
		} pressure;

		struct __attribute((packed))
		{
			uint16_t tag_id;
			uint8_t rssi;
		} rssi;
	};
} rtls_status_msg_pkt_t;

#endif /* RTLS_MESSAGES_H__ */
