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

#ifndef RTLS_CONTROL_CLIENT_H__
#define RTLS_CONTROL_CLIENT_H__


#include <stdint.h>
#include "access.h"
#include "access_reliable.h"
#include "rtls_control_common.h"
#include "rtls_control_messages.h"

#define RTLS_CONTROL_CLIENT_MODEL_ID 0x1003

typedef struct __rtls_control_client_t rtls_control_client_t;

typedef void (*rtls_state_status_cb_t)(const rtls_control_client_t * p_self,
                                                const access_message_rx_meta_t * p_meta,
                                                const rtls_control_status_params_t * p_in);

typedef struct
{
    rtls_state_status_cb_t rtls_status_cb;
    access_reliable_cb_t ack_transaction_status_cb;
} rtls_control_client_callbacks_t;

typedef struct
{
    uint32_t timeout;
    bool force_segmented;
    nrf_mesh_transmic_size_t transmic_size;
    const rtls_control_client_callbacks_t * p_callbacks;
} rtls_control_client_settings_t;

typedef union
{
    rtls_control_set_msg_pkt_t set;
} rtls_control_client_msg_data_t;

struct __rtls_control_client_t
{
    access_model_handle_t model_handle;
    rtls_control_client_msg_data_t msg_pkt;
    access_reliable_t access_message;
    rtls_control_client_settings_t settings;
};

uint32_t rtls_control_client_init(rtls_control_client_t * p_client, uint8_t element_index);

uint32_t rtls_control_client_get(rtls_control_client_t * p_client);

#endif /* RTLS_CLIENT_H__ */
