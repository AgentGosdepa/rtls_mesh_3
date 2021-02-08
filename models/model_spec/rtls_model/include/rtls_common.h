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

#ifndef RTLS_COMMON_H__
#define RTLS_COMMON_H__

#include <stdint.h>
#include "model_common.h"

#define RTLS_COMPANY_ID 0xFFFF
#define RTLS_MAX        (0x01)

#define RTLS_PULSE_TYPE 1
#define RTLS_PRESSURE_TYPE 2
#define RTLS_RSSI_TYPE 3

typedef struct
{
	struct
	{
		uint8_t pressure_up;
		uint8_t pressure_down;
	} pressure;

	struct
	{
		uint16_t tag_id;
		uint8_t rssi;
	} rssi;

	uint8_t pulse;
} rtls_state_t;

typedef struct
{
	union
	{
		struct
		{
			uint8_t pressure_up;
			uint8_t pressure_down;
		} pressure;

		struct
		{
			uint16_t tag_id;
			uint8_t rssi;
		} rssi;
		uint8_t pulse;
	};
	uint8_t type;
} rtls_set_params_t;

typedef struct
{
	union
	{
		struct
		{
			uint8_t pressure_up;
			uint8_t pressure_down;
		} pressure;

		struct
		{
			uint16_t tag_id;
			uint8_t rssi;
		} rssi;
		uint8_t pulse;
	};
	uint8_t type;
} rtls_status_params_t;

#endif /* RTLS_COMMON_H__ */
