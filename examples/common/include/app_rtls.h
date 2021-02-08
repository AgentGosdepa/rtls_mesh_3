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


/**
 * @defgroup APP_ONOFF Generic OnOff server behaviour
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Application level OnOff server behavioral structures, functions, and callbacks.
 *
 * This module implements the behavioral requirements of the Generic OnOff server model.
 *
 * The application should use the set/transition callback provided by this module to set the
 * hardware state. The hardware state could be changed by reflecting the value provided by the
 * set/transiton_time callback on the GPIO or by sending this value to the connected lighting
 * peripheral using some other interface (e.g. serial interface). Similarly, the application should
 * use the get callback provided by this module to read the hardware state.
 *
 * This module triggers the set/transition callback only when it determins that it is time to
 * inform the user application. It is possible that the client can send multiple overlapping
 * set/transition commands. In such case any transition in progress will be abandoned and fresh
 * transition will be started if required.
 *
 * Using transition_cb:
 * If the underlaying hardware does not support setting of the instantaneous value provided via
 * `set_cb`, the `transition_cb` can be used to implement the transition effect according to
 * provided transition parameters. This callback will be called when transition start with the
 * required transition time and target value. When the transition is complete this callback will be
 * called again with transition time set to 0 and the desired target value.
 * <br>
 * @warning To comply with the @tagMeshMdlSp test cases, the application must adhere to
 * the requirements defined in the following sections:
 * - @tagMeshMdlSp section 3.1.1 (Generic OnOff) and section 3.3.1.2 (Generic OnOff state behaviour).
 * - @tagMeshSp section 3.7.6.1 (Publish).
 *
 * These requirements are documented at appropriate places in the module source code.
 *
 * @{
 */

/**
 * Macro to create application level app_onoff_server_t context.
 *
 * Individual timer instances are created for each model instance.
 *
 * @param[in] _name                 Name of the app_onoff_server_t instance
 * @param[in] _force_segmented      If the Generic OnOff server shall use force segmentation of messages
 * @param[in] _mic_size             MIC size to be used by Generic OnOff server
 * @param[in] _set_cb               Callback for setting the application state to given value.
 * @param[in] _get_cb               Callback for reading the state from the application.
 * @param[in] _transition_cb        Callback for setting the application transition time and state value to given values.
*/
#define APP_RTLS_SERVER_DEF(_name, _force_segmented, _mic_size, _set_cb)  \
    APP_TIMER_DEF(_name ## _timer); \
    static app_rtls_server_t _name =  \
    {  \
        .server.settings.force_segmented = _force_segmented,  \
        .server.settings.transmic_size = _mic_size,  \
        .rtls_set_cb = _set_cb,  \
    };


/** Internal structure to hold state and timing information. */
typedef union
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
} app_rtls_state_t;

/* Forward declaration */
typedef struct __app_rtls_server_t app_rtls_server_t;

typedef void (*app_rtls_set_cb_t)(const app_rtls_server_t * p_app, const rtls_set_params_t * set_data, 
                                                            const access_message_rx_meta_t * p_meta);

/** Application level structure holding the OnOff server model context and OnOff state representation */
struct __app_rtls_server_t
{
    /** OnOff server model interface context structure */
    rtls_server_t server;
    /** APP timer instance pointer */
    app_timer_id_t const * p_timer_id;
    /** Callaback to be called for informing the user application to update the value*/
    app_rtls_set_cb_t  rtls_set_cb;

    /** Internal variable. Representation of the OnOff state related data and transition parameters
     *  required for behavioral implementation, and for communicating with the application */
    app_rtls_state_t state;
    /** Internal variable. It is used for acquiring RTC counter value. */
    uint32_t last_rtc_counter;
    /** Internal variable. To flag if the received message has been processed to update the present
     * OnOff value */
    bool value_updated;
};

/** Initializes the behavioral module for the generic OnOff model
 *
 * @param[in] p_app                 Pointer to [app_onoff_server_t](@ref __app_onoff_server_t)
 *                                  context.
 * @param[in] element_index         Element index on which this server will be instantiated.
 *
 * @retval NRF_SUCCESS              If initialization is successful.
 * @retval NRF_ERROR_NO_MEM         @ref ACCESS_MODEL_COUNT number of models already allocated, or
 *                                  no more subscription lists available in memory pool.
 * @retval NRF_ERROR_NULL           NULL pointer is supplied to the function or to the required
 *                                  member variable pointers.
 * @retval NRF_ERROR_NOT_FOUND      Invalid access element index, or access handle invalid.
 * @retval NRF_ERROR_FORBIDDEN      Multiple model instances per element are not allowed or changes
 *                                  to device composition are not allowed. Adding a new model after
 *                                  device is provisioned is not allowed.
 * @retval  NRF_ERROR_INVALID_PARAM Model not bound to appkey, publish address not set or wrong
 *                                  opcode format. The application timer module has not been
 *                                  initialized or timeout handler is not provided.
 * @retval NRF_ERROR_INVALID_STATE  If the application timer is running.
*/
uint32_t app_rtls_init(app_rtls_server_t * p_app, uint8_t element_index);

/** @} end of APP_ONOFF */
#endif /* APP_RTLS_H__ */
