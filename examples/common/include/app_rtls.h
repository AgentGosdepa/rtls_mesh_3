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

#ifndef APP_RTLS_H__
#define APP_RTLS_H__

#include <stdint.h>

#include "rtls_server.h"
#include "app_timer.h"

#define APP_RTLS_SERVER_DEF(_name, _force_segmented, _mic_size, _set_cb)  \
    APP_TIMER_DEF(_name ## _timer); \
    static app_rtls_server_t _name =  \
    {  \
        .server.settings.force_segmented = _force_segmented,  \
        .server.settings.transmic_size = _mic_size,  \
        .rtls_set_cb = _set_cb  \
    };

typedef union
{
    uint8_t pulse;

    struct
    {
        uint8_t pressure_up;
        uint8_t pressure_down;
    } pressure;

    struct
    {
        uint8_t tag_id[6];
        uint8_t rssi;
    } rssi;
} app_rtls_state_t;

/* Forward declaration */
typedef struct __app_rtls_server_t app_rtls_server_t;

typedef void (*app_rtls_set_cb_t)(const app_rtls_server_t * p_app, const rtls_set_params_t * set_data, 
                                                            const access_message_rx_meta_t * p_meta);
/** Application level structure holding the OnOff server model context and OnOff state representation */
struct __app_rtls_server_t
{
    rtls_server_t server;
    app_timer_id_t const * p_timer_id;
    app_rtls_set_cb_t  rtls_set_cb;
    app_rtls_state_t state;
};

uint32_t app_rtls_init(app_rtls_server_t * p_app, uint8_t element_index);

/** @} end of APP_ONOFF */
#endif /* APP_RTLS_H__ */
