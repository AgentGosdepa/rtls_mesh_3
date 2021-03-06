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

#include <stdint.h>
#include <string.h>

/* HAL */
#include "boards.h"
#include "simple_hal.h"
#include "app_timer.h"

/* Core */
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "rtls_client.h"
#include "rtls_control_server.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
//#include "sdk_config.h"
#include "nrf_mesh_config_app.h"
#include "uri.h"
#include "ble_softdevice_support.h"

//#include "app_config.h"
//#include "nrf_mesh_config_examples.h"
//#include "light_switch_example_common.h"
//#include "example_common.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/
#define APP_UNACK_MSG_REPEAT_COUNT   (2)

/* Controls if the model instance should force all mesh messages to be segmented messages. */
#define APP_FORCE_SEGMENTATION       (false)
/* Controls the MIC size used by the model instance for sending the mesh messages. */
#define APP_MIC_SIZE                 (NRF_MESH_TRANSMIC_SIZE_SMALL)

#if 0
#define DONGLE
#else
#define BEACON
#endif

/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
#ifdef DONGLE
#include "rtls_rssi_client.h"
static void app_rtls_rssi_client_status_cb(const rtls_rssi_client_t * p_self, 
                                                        const access_message_rx_meta_t * p_meta);
static void app_rtls_rssi_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status);

static void app_rtls_rssi_client_periodic_cb(access_model_handle_t handle, void * p_args);
#endif

#ifdef BEACON
#include "rtls_rssi_server.h"
static void app_rtls_rssi_server_set_cb_t(const rtls_rssi_server_t * p_self,
                                             const access_message_rx_meta_t * p_meta);
#endif

static void app_rtls_client_status_cb(const rtls_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const rtls_status_params_t * p_in);

static void app_rtls_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status);

static void app_rtls_control_client_get_cb_t(const rtls_control_server_t * p_self,
                                             const access_message_rx_meta_t * p_meta,
                                             rtls_control_status_params_t * p_out);

/*****************************************************************************
 * Static variables
 *****************************************************************************/
#ifdef DONGLE
static rtls_rssi_client_t m_rssi_clients[1];
const rtls_rssi_client_callbacks_t rssi_client_cbs =
{
    .rtls_status_cb = app_rtls_rssi_client_status_cb,
    .ack_transaction_status_cb = app_rtls_rssi_client_transaction_status_cb,
    .periodic_publish_cb = app_rtls_rssi_client_periodic_cb
};
#endif

#ifdef BEACON
static rtls_rssi_server_t m_rssi_servers[1];
const rtls_rssi_server_callbacks_t rssi_server_cbs =
{
    .rtls_cbs.set_cb = app_rtls_rssi_server_set_cb_t
};
#endif

static rtls_client_t m_clients[1];
static rtls_control_server_t m_control_servers[1];
static bool                   m_device_provisioned;

const rtls_client_callbacks_t client_cbs =
{
    .rtls_status_cb = app_rtls_client_status_cb,
    .ack_transaction_status_cb = app_rtls_client_transaction_status_cb
};

const rtls_control_server_callbacks_t control_server_cbs =
{
    .rtls_cbs.get_cb = app_rtls_control_client_get_cb_t
};

static void device_identification_start_cb(uint8_t attention_duration_s)
{
    hal_led_mask_set(LEDS_MASK, false);
    hal_led_blink_ms(BSP_LED_2_MASK  | BSP_LED_3_MASK,
                     LED_BLINK_ATTENTION_INTERVAL_MS,
                     LED_BLINK_ATTENTION_COUNT(attention_duration_s));
}

static void provisioning_aborted_cb(void)
{
    hal_led_blink_stop();
}

static void unicast_address_print(void)
{
    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

#if MESH_FEATURE_GATT_ENABLED
    /* Restores the application parameters after switching from the Provisioning
     * service to the Proxy  */
    gap_params_init();
    conn_params_init();
#endif

    unicast_address_print();
    hal_led_blink_stop();
    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}


#ifdef DONGLE
static void app_rtls_rssi_client_status_cb(const rtls_rssi_client_t * p_self, 
                                                        const access_message_rx_meta_t * p_meta)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Data was delivered from 0x%x to 0x%x.\n", p_meta->dst.value, p_meta->src.value);
}

static void app_rtls_rssi_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status)
{
    switch(status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer success.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer timeout.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer cancelled.\n");
            break;

        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }
}

static void app_rtls_rssi_client_periodic_cb(access_model_handle_t handle, void * p_args)
{
    rtls_rssi_client_set_unack(m_rssi_clients, 2);
}

#endif

#ifdef BEACON
static void app_rtls_rssi_server_set_cb_t(const rtls_rssi_server_t * p_self,
                                             const access_message_rx_meta_t * p_meta)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Data was delivered from 0x%x to 0x%x, rssi is %d = 0x%x.\n", 
                        p_meta->src.value, p_meta->dst.value, p_meta->p_core_metadata->params.scanner.rssi, p_meta->p_core_metadata->params.scanner.rssi);

    rtls_set_params_t msg = 
    {
        .rssi.rssi = p_meta->p_core_metadata->params.scanner.rssi,
        .rssi.tag_id = p_meta->src.value,
        .type = RTLS_RSSI_TYPE
    };

    rtls_client_set(m_clients, &msg, NULL);
    //rtls_client_set_unack(m_clients, &msg, NULL, 2);
}
#endif

static void app_rtls_client_status_cb(const rtls_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const rtls_status_params_t * p_in)
{
    if (p_in->type == RTLS_PULSE_TYPE)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transfering data: 0x%x, to: 0x%x complete.\n", p_in->pulse, p_meta->src.value);
    }
    else if (p_in->type == RTLS_PRESSURE_TYPE)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transfering data: 0x%x|0x%x to: 0x%x complete.\n", p_in->pressure.pressure_up, 
                                            p_in->pressure.pressure_down, p_meta->src.value);
    }
    else if (p_in->type == RTLS_RSSI_TYPE)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transfering data: rssi = 0x%x from 0x%x, addr = 0x%x complete.\n", 
                                            p_in->rssi.rssi, p_meta->src.value,
                                            p_in->rssi.tag_id);
    }
}

/* Acknowledged transaction status callback, if acknowledged transfer fails, application can
* determine suitable course of action (e.g. re-initiate previous transaction) by using this
* callback.
*/
static void app_rtls_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status)
{
    switch(status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer success.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer timeout.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer cancelled.\n");
            break;

        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }
}

static void app_rtls_control_client_get_cb_t(const rtls_control_server_t * p_self, const access_message_rx_meta_t * p_meta,
                                             rtls_control_status_params_t * p_out)
{
    p_out->type = RTLS_UUID_TYPE;
    p_out->uuid.tag_id[0] = 0x44;
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

#if NRF_MESH_LOG_ENABLE
static const char m_usage_string[] =
    "\n"
    "\t\t------------------------------------------------------------------------------------\n";
#endif

static void button_event_handler(uint32_t button_number)
{
    /* Increase button number because the buttons on the board is marked with 1 to 4 */


    button_number++;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    uint32_t status = NRF_SUCCESS;
    rtls_set_params_t set_params;
    
    switch(button_number)
    {
        case 1:
            set_params.type = RTLS_PULSE_TYPE;
            set_params.pulse = 0xFB;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transfering data: 0x%x start.\n", set_params.pulse);
            break;

        case 2:
            set_params.type = RTLS_PRESSURE_TYPE;
            set_params.pressure.pressure_up = 0xB2;
            set_params.pressure.pressure_down = 0x2B;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transfering data: 0x%x|0x%x start.\n", set_params.pressure.pressure_up, 
                                            set_params.pressure.pressure_down);
            break;
#ifdef BEACON
        case 3:

            set_params.type = RTLS_RSSI_TYPE;
            set_params.rssi.rssi = 0xC6;
            set_params.rssi.tag_id = 0xBBAA;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transfering data: rssi = 0x%x addr = 0x%x start.\n", 
                                            set_params.rssi.rssi,
                                            set_params.rssi.tag_id);
            break;
        case 4:
            if (0)
            {
            set_params.type = RTLS_RSSI_TYPE;
            set_params.rssi.rssi = 0xAA;
            set_params.rssi.tag_id = 0xAABB;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transfering data: rssi = 0x%x addr = 0x%x start.\n", 
                                            set_params.rssi.rssi,
                                            set_params.rssi.tag_id);
            }
            break;
#endif
    }

    switch (button_number)
    {
        case 1:
            status = rtls_client_set_unack(&m_clients[0], &set_params, NULL, 2);
            hal_led_blink_ms(BSP_LED_3, 200, 2);
            break;

        case 2:
            (void)access_model_reliable_cancel(m_clients[0].model_handle);
            status = rtls_client_set_unack(&m_clients[0], &set_params, NULL, 2);
            hal_led_blink_ms(BSP_LED_3, 200, 2);
            break;

        case 3:
#ifdef BEACON
            status = rtls_client_set_unack(&m_clients[0], &set_params, NULL, 2);
            hal_led_blink_ms(BSP_LED_3, 200, 2);

#else
            status = rtls_rssi_client_set(m_rssi_clients);
            hal_led_blink_ms(BSP_LED_3, 200, 2);
#endif
            break;
        case 4:
        status = NRF_SUCCESS;
            if (mesh_stack_is_device_provisioned())
            {
                #if MESH_FEATURE_GATT_PROXY_ENABLED
                (void) proxy_stop();
                #endif
                mesh_stack_config_clear();
                node_reset();
            }
            else
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "The device is unprovisioned. Resetting has no effect.\n");
            }

        if (0)
        {
#ifdef BEACON
            (void)access_model_reliable_cancel(m_clients[0].model_handle);
            status = rtls_client_set(&m_clients[0], &set_params, NULL);
            hal_led_blink_ms(BSP_LED_3, 200, 2);
#else
            status = rtls_rssi_client_set_unack(m_rssi_clients, 2);
            hal_led_blink_ms(BSP_LED_3, 200, 2);
#endif
        }
            break;
        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
            break;
    }

    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_NO_MEM:
        case NRF_ERROR_BUSY:
        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Client %u cannot send\n", button_number);
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            break;

        case NRF_ERROR_INVALID_PARAM:
            /* Publication not enabled for this client. One (or more) of the following is wrong:
             * - An application key is missing, or there is no application key bound to the model
             * - The client does not have its publication state set
             *
             * It is the provisioner that adds an application key, binds it to the model and sets
             * the model's publication state.
             */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication not configured for client %u\n", button_number);
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}

static void rtt_input_handler(int key)
{
    if (key >= '1' && key <= '4')
    {
        uint32_t button_number = key - '1';
        button_event_handler(button_number);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
    }
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");

    for (uint32_t i = 0; i < 1; ++i)
    {
        m_clients[i].settings.p_callbacks = &client_cbs;
        m_clients[i].settings.timeout = 0;
        m_clients[i].settings.force_segmented = APP_FORCE_SEGMENTATION;
        m_clients[i].settings.transmic_size = APP_MIC_SIZE;

        m_control_servers[i].settings.p_callbacks = &control_server_cbs;
        m_control_servers[i].settings.timeout = 0;
        m_control_servers[i].settings.force_segmented = APP_FORCE_SEGMENTATION;
        m_control_servers[i].settings.transmic_size = APP_MIC_SIZE;


#ifdef DONGLE
        m_rssi_clients[i].settings.p_callbacks = &rssi_client_cbs;
        m_rssi_clients[i].settings.timeout = 0;
        m_rssi_clients[i].settings.force_segmented = APP_FORCE_SEGMENTATION;
        m_rssi_clients[i].settings.transmic_size = APP_MIC_SIZE;
        ERROR_CHECK(rtls_rssi_client_init(&m_rssi_clients[i], i));
#endif

#ifdef BEACON
        m_rssi_servers[i].settings.p_callbacks = &rssi_server_cbs;
        m_rssi_servers[i].settings.force_segmented = APP_FORCE_SEGMENTATION;
        m_rssi_servers[i].settings.transmic_size = APP_MIC_SIZE;
        ERROR_CHECK(rtls_rssi_server_init(&m_rssi_servers[i], i));
#endif

        ERROR_CHECK(rtls_client_init(&m_clients[i], i));
        ERROR_CHECK(rtls_control_server_init(&m_control_servers[i], i));
    }
}

static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };

    uint32_t status = mesh_stack_init(&init_params, &m_device_provisioned);
    switch (status)
    {
        case NRF_ERROR_INVALID_DATA:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Data in the persistent memory was corrupted. Device starts as unprovisioned.\n");
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Reset device before starting of the provisioning process.\n");
            break;
        case NRF_SUCCESS:
            break;
        default:
            ERROR_CHECK(status);
    }
}

static void initialize(void)
{
    //__LOG_INIT(0, 0, LOG_CALLBACK_DEFAULT);
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS | LOG_SRC_BEARER, LOG_LEVEL_INFO | LOG_LEVEL_DBG1, LOG_CALLBACK_DEFAULT);
    //__LOG_INIT(LOG_SRC_APP | LOG_SRC_FRIEND, LOG_LEVEL_DBG1, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Client Demo -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif

    ble_stack_init();

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    mesh_init();
}

static void start(void)
{
    rtt_input_enable(rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_sd_ble_opt_set_cb = NULL,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = device_identification_start_cb,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = provisioning_aborted_cb,
#ifdef DONGLE
            .p_device_uri = EX_URI_RTLS_DONGLE
#elif defined BEACON
            .p_device_uri = EX_URI_RTLS_BEACON
#else
            .p_device_uri = EX_URI_LS_CLIENT
#endif
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
    else
    {
        unicast_address_print();
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}

int main(void)
{
    initialize();
    start();

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
