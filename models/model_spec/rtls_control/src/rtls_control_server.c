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

#include "rtls_control_server.h"
#include "rtls_control_common.h"
#include "rtls_control_messages.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "access.h"
#include "access_config.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"
#include "nordic_common.h"

#include "log.h"

static uint32_t status_send(rtls_control_server_t * p_server,
                            const access_message_rx_t * p_message,
                            const rtls_control_status_params_t * p_params)
{
    rtls_control_set_msg_pkt_t msg_pkt;
    uint16_t length = 0;

    if (p_params->type == RTLS_UUID_TYPE)
    {
        for(uint8_t i = 0; i < 6; i++)
        {
            msg_pkt.uuid.tag_id[i] = p_params->uuid.tag_id[i];
        }
        length = RTLS_UUID_GET_LEN;
    }
__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "status_send: type = %d\n", p_params->type);
    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(RTLS_OPCODE_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = length,
        .force_segmented = p_server->settings.force_segmented,
        .transmic_size = p_server->settings.transmic_size
    };

    return access_model_reply(p_server->model_handle, p_message, &reply);
}

/** Opcode Handlers */

static void handle_get(access_model_handle_t model_handle, const access_message_rx_t * p_rx_msg, void * p_args)
{
    rtls_control_server_t * p_server = (rtls_control_server_t *) p_args;
    rtls_control_status_params_t out_data = {0};

__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "handle_get: from %x to %x\n", p_rx_msg->meta_data.src.value, p_rx_msg->meta_data.dst.value);

    p_server->settings.p_callbacks->rtls_cbs.get_cb(p_server, &p_rx_msg->meta_data, &out_data);
    (void) status_send(p_server, p_rx_msg, &out_data);
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_SIG(RTLS_OPCODE_UUID_GET), handle_get}
};


/** Interface functions */
uint32_t rtls_control_server_init(rtls_control_server_t * p_server, uint8_t element_index)
{
    if (p_server == NULL ||
        p_server->settings.p_callbacks == NULL ||
        p_server->settings.p_callbacks->rtls_cbs.get_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

    access_model_add_params_t init_params =
    {
        .model_id = ACCESS_MODEL_SIG(RTLS_CONTROL_SERVER_MODEL_ID),
        .element_index =  element_index,
        .p_opcode_handlers = &m_opcode_handlers[0],
        .opcode_count = ARRAY_SIZE(m_opcode_handlers),
        .p_args = p_server,
        .publish_timeout_cb = NULL
    };

    uint32_t status = access_model_add(&init_params, &p_server->model_handle);

    if (status == NRF_SUCCESS)
    {
        status = access_model_subscription_list_alloc(p_server->model_handle);
    }

    return status;
}

uint32_t rtls_server_status_publish(rtls_control_server_t * p_server, const rtls_control_status_params_t * p_params)
{
    if (p_server == NULL ||
        p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    return status_send(p_server, NULL, p_params);
}