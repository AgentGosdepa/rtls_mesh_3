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

#include "app_rtls.h"

#include <stdint.h>

#include "utils.h"
#include "sdk_config.h"
#include "example_common.h"
#include "rtls_server.h"

#include "log.h"
#include "app_timer.h"

/** This sample implementation shows how the model behavior requirements of Generic rtls server can
 * be implemented.
 */

/* Forward declaration */
static void rtls_state_get_cb(const rtls_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       rtls_status_params_t * p_out);
static void rtls_state_set_cb(const rtls_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       const rtls_set_params_t * p_in,
                                       const model_transition_t * p_in_transition,
                                       rtls_status_params_t * p_out);

const rtls_server_callbacks_t rtls_srv_cbs =
{
    .rtls_cbs.set_cb = rtls_state_set_cb,
    .rtls_cbs.get_cb = rtls_state_get_cb
};

/***** Generic rtls model interface callbacks *****/

static void rtls_state_get_cb(const rtls_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       rtls_status_params_t * p_out)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "msg: GET \n");

    app_rtls_server_t   * p_app = PARENT_BY_FIELD_GET(app_rtls_server_t, server, p_self);

    /* Requirement: Provide the current value of the rtls state */
    const uint8_t set_len = 10;
    uint8_t set_data[10] = {0};
    p_app->rtls_get_cb(p_app, set_data, set_len);

    for(int i = 0; i < 6; i++)
      p_app->state.smartband_id[i] = set_data[i];
    for(int i = 6; i < 9; i++)
      p_app->state.smartband_data[i-6] = set_data[i];
    p_app->state.rssi = set_data[9];

    for(int i = 0; i < 6; i++)
      p_out->smartband_id[i] = p_app->state.smartband_id[i]; 
    for(int i = 0; i < 3; i++)
      p_out->smartband_data[i] = p_app->state.smartband_data[i];
    p_out->rssi = p_app->state.rssi;
}

static void rtls_state_set_cb(const rtls_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       const rtls_set_params_t * p_in,
                                       const model_transition_t * p_in_transition,
                                       rtls_status_params_t * p_out)
{
    app_rtls_server_t   * p_app = PARENT_BY_FIELD_GET(app_rtls_server_t, server, p_self);

    const uint8_t set_len = 10;
    uint8_t set_data[10] = {0};

    for(int i = 0; i < 6; i++)
      set_data[i] = p_in->smartband_id[i];
    for(int i = 0; i < 3; i++)
      set_data[i+6] = p_in->smartband_data[i];
    set_data[9] = p_in->rssi;

    p_app->rtls_set_cb(p_app, set_data, set_len);

    /* Prepare response */
    if (p_out != NULL)
    {
      for(int i = 0; i < 6; i++)
        p_out->smartband_id[i] = p_app->state.smartband_id[i]; 
      for(int i = 0; i < 3; i++)
        p_out->smartband_data[i] = p_app->state.smartband_data[i];
      p_out->rssi = p_app->state.rssi;
    }
}

/***** Interface functions *****/

void app_rtls_status_publish(app_rtls_server_t * p_app)
{
    const uint8_t set_len = 10;
    uint8_t set_data[10] = {0};
    p_app->rtls_get_cb(p_app, set_data, set_len);

    for(int i = 0; i < 6; i++)
      p_app->state.smartband_id[i] = set_data[i];
    for(int i = 6; i < 9; i++)
      p_app->state.smartband_data[i-6] = set_data[i];
    p_app->state.rssi = set_data[9];


    rtls_status_params_t status;
    for(int i = 0; i < 6; i++)
      status.smartband_id[i] = p_app->state.smartband_id[i];
    
    for(int i = 0; i < 3; i++)
      status.smartband_data[i] = p_app->state.smartband_data[i];

    status.rssi = p_app->state.rssi;
    (void) rtls_server_status_publish(&p_app->server, &status);
}

uint32_t app_rtls_init(app_rtls_server_t * p_app, uint8_t element_index)
{
    uint32_t status = NRF_ERROR_INTERNAL;

    if (p_app == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_app->server.settings.p_callbacks = &rtls_srv_cbs;
    if ( (p_app->rtls_get_cb == NULL) ||
         ( (p_app->rtls_set_cb == NULL) ) )
    {
        return NRF_ERROR_NULL;
    }

    status = rtls_server_init(&p_app->server, element_index);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    return NRF_SUCCESS;
}
