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

#include "rtls_client.h"
#include "model_common.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "access.h"
#include "access_config.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"
#include "nordic_common.h"

#include "log.h"

static void status_handle(access_model_handle_t handle, const access_message_rx_t * p_rx_msg, void * p_args)
{
    rtls_client_t * p_client = (rtls_client_t *) p_args;
    rtls_status_params_t in_data = {0};

    rtls_status_msg_pkt_t * p_msg_params_packed = (rtls_status_msg_pkt_t *) p_rx_msg->p_data;

    if (p_rx_msg->length == RTLS_PULSE_SET_LEN)
    {
        in_data.pulse = p_msg_params_packed->pulse;
        in_data.type = RTLS_PULSE_TYPE;
    }
    else if (p_rx_msg->length == RTLS_PRESSURE_SET_LEN)
    {
        in_data.pressure.pressure_up = p_msg_params_packed->pressure.pressure_up;
        in_data.pressure.pressure_down = p_msg_params_packed->pressure.pressure_down;
        in_data.type = RTLS_PRESSURE_TYPE;
    }
    else if (p_rx_msg->length == RTLS_RSSI_SET_LEN)
    {
        in_data.rssi.rssi = p_msg_params_packed->rssi.rssi;
        for(uint8_t i = 0; i < 6; i++)
        {
            in_data.rssi.tag_id[i] = p_msg_params_packed->rssi.tag_id[i];
        }
        in_data.type = RTLS_RSSI_TYPE;
    }

    p_client->settings.p_callbacks->rtls_status_cb(p_client, &p_rx_msg->meta_data, &in_data);
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_SIG(RTLS_OPCODE_STATUS), status_handle},
};

static uint8_t message_set_packet_create(rtls_set_msg_pkt_t *p_set, const rtls_set_params_t * p_params,
                                      const model_transition_t * p_transition)
{
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Current type: %x\n", p_params->type);
    if (p_params->type == RTLS_PULSE_TYPE)
    {
        p_set->pulse = p_params->pulse;
        return RTLS_PULSE_SET_LEN;
    }
    else if (p_params->type == RTLS_PRESSURE_TYPE)
    {
        p_set->pressure.pressure_up = p_params->pressure.pressure_up;
        p_set->pressure.pressure_down = p_params->pressure.pressure_down;
        return RTLS_PRESSURE_SET_LEN;
    }
    else if (p_params->type == RTLS_RSSI_TYPE)
    {
        p_set->rssi.rssi = p_params->rssi.rssi;
        for(uint8_t i = 0; i < 6; i++)
        {
            p_set->rssi.tag_id[i] = p_params->rssi.tag_id[i];
        }
        return RTLS_RSSI_SET_LEN;
    }
    else
    {
        NRF_MESH_ASSERT(0);
    }
}

static void message_create(rtls_client_t * p_client, uint16_t tx_opcode, const uint8_t * p_buffer,
                           uint16_t length, access_message_tx_t *p_message)
{
    p_message->opcode.opcode = tx_opcode;
    p_message->opcode.company_id = ACCESS_COMPANY_ID_NONE;
    p_message->p_buffer = p_buffer;
    p_message->length = length;
    p_message->force_segmented = p_client->settings.force_segmented;
    p_message->transmic_size = p_client->settings.transmic_size;
    p_message->access_token = nrf_mesh_unique_token_get();
}

static void reliable_context_create(rtls_client_t * p_client, uint16_t reply_opcode,
                                    access_reliable_t * p_reliable)
{
    p_reliable->model_handle = p_client->model_handle;
    p_reliable->reply_opcode.opcode = reply_opcode;
    p_reliable->reply_opcode.company_id = ACCESS_COMPANY_ID_NONE;
    p_reliable->timeout = p_client->settings.timeout;
    p_reliable->status_cb = p_client->settings.p_callbacks->ack_transaction_status_cb;
}

/** Interface functions */

uint32_t rtls_client_init(rtls_client_t * p_client, uint8_t element_index)
{
    if (p_client == NULL ||
        p_client->settings.p_callbacks == NULL ||
        p_client->settings.p_callbacks->rtls_status_cb == NULL ||
        p_client->settings.p_callbacks->ack_transaction_status_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_client->settings.timeout == 0)
    {
        p_client->settings.timeout= MODEL_ACKNOWLEDGED_TRANSACTION_TIMEOUT;
    }

    access_model_add_params_t add_params =
    {
        .model_id = ACCESS_MODEL_SIG(RTLS_CLIENT_MODEL_ID),
        .element_index = element_index,
        .p_opcode_handlers = &m_opcode_handlers[0],
        .opcode_count = ARRAY_SIZE(m_opcode_handlers),
        .p_args = p_client,
        .publish_timeout_cb = NULL
    };

    uint32_t status = access_model_add(&add_params, &p_client->model_handle);

    if (status == NRF_SUCCESS)
    {
        status = access_model_subscription_list_alloc(p_client->model_handle);
    }

    return status;
}

uint32_t rtls_client_set(rtls_client_t * p_client, const rtls_set_params_t * p_params,
                                  const model_transition_t * p_transition)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        uint8_t server_msg_length = message_set_packet_create(&p_client->msg_pkt.set, p_params, p_transition);
        

        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending len: %d\n", server_msg_length);
        __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: ", (const uint8_t *)&p_client->msg_pkt.set, server_msg_length);

        uint16_t opcode = 0;
        if (server_msg_length == RTLS_PULSE_SET_LEN)
        {
            opcode = RTLS_OPCODE_PULSE_SET;
        }
        else if (server_msg_length == RTLS_PRESSURE_SET_LEN)
        {
            opcode = RTLS_OPCODE_PRESSURE_SET;
        }
        else if (server_msg_length == RTLS_RSSI_SET_LEN)
        {
            opcode = RTLS_OPCODE_RSSI_SET;
        }
        NRF_MESH_ASSERT(opcode);

        message_create(p_client, opcode, (const uint8_t *) &p_client->msg_pkt.set,
                       server_msg_length, &p_client->access_message.message);
        reliable_context_create(p_client, RTLS_OPCODE_STATUS, &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}

uint32_t rtls_client_set_unack(rtls_client_t * p_client, const rtls_set_params_t * p_params,
                                        const model_transition_t * p_transition, uint8_t repeats)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    rtls_set_msg_pkt_t msg;
    uint8_t server_msg_length = message_set_packet_create(&msg, p_params, p_transition);

    uint16_t opcode = 0;
    if (server_msg_length == RTLS_PULSE_SET_LEN)
    {
        opcode = RTLS_OPCODE_PULSE_SET_UNACKNOWLEDGED;
    }
    else if (server_msg_length == RTLS_PRESSURE_SET_LEN)
    {
        opcode = RTLS_OPCODE_PRESSURE_SET_UNACKNOWLEDGED;
    }
    else if (server_msg_length == RTLS_RSSI_SET_LEN)
    {
        opcode = RTLS_OPCODE_RSSI_SET_UNACKNOWLEDGED;
    }
    NRF_MESH_ASSERT(opcode);

    message_create(p_client, opcode, (const uint8_t *) &msg, server_msg_length, &p_client->access_message.message);

    uint32_t status = NRF_SUCCESS;
    repeats++;
    while (repeats-- > 0 && status == NRF_SUCCESS)
    {
        status = access_model_publish(p_client->model_handle, &p_client->access_message.message);
    }
    return status;
}


