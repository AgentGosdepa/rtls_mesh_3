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
#include "proxy.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "rtls_server.h"
#include "rtls_control_client.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"
#include "nrf_mesh_serial.h"

/* Example specific includes */
#include "app_rtls.h"
#include "app_config.h"
#include "example_common.h"
#include "nrf_mesh_config_examples.h"
#include "light_switch_example_common.h"
#include "ble_softdevice_support.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/
#define RTLS_SERVER_0_LED          (BSP_LED_0)
#define APP_RTLS_ELEMENT_INDEX     (0)

/* Controls if the model instance should force all mesh messages to be segmented messages. */
#define APP_FORCE_SEGMENTATION      (false)
/* Controls the MIC size used by the model instance for sending the mesh messages. */
#define APP_MIC_SIZE                (NRF_MESH_TRANSMIC_SIZE_SMALL)

#define EX_URI_RTLS_PC URI_SCHEME_EXAMPLE "URI for RTLS_PC example"
/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
static void app_rtls_server_set_cb(const app_rtls_server_t * p_app, const rtls_set_params_t * set_data,
                                                                const access_message_rx_meta_t * p_meta);

static void app_rtls_control_client_status_cb(const rtls_control_client_t * p_self, const access_message_rx_meta_t * p_meta,
                                                const rtls_control_status_params_t * p_in);

static void app_rtls_control_client_transaction_status_cb(access_model_handle_t model_handle, void * p_args,
                                                access_reliable_status_t status);

/*****************************************************************************
 * Static variables
 *****************************************************************************/
static bool m_device_provisioned;

/* Generic OnOff server structure definition and initialization */
APP_RTLS_SERVER_DEF(m_rtls_server_0,
                     APP_FORCE_SEGMENTATION,
                     APP_MIC_SIZE,
                     app_rtls_server_set_cb
                     )

static rtls_control_client_t m_control_clients[1];

const rtls_control_client_callbacks_t client_cbs =
{
    .rtls_status_cb = app_rtls_control_client_status_cb,
    .ack_transaction_status_cb = app_rtls_control_client_transaction_status_cb
};

static uint8_t uart_buff1[9], uart_buff2[8];
static uint8_t uart_len1, uart_len2;
/* Callback for updating the hardware state */
static void app_rtls_server_set_cb(const app_rtls_server_t * p_app, const rtls_set_params_t * set_data, 
                                                                const access_message_rx_meta_t * p_meta)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "current type %x.\n", set_data->type);
    if (set_data->type == RTLS_PULSE_TYPE)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Coming data: type - 0x%x, from: 0x%x, 0x%x complete.\n", 
                                            set_data->type, p_meta->src.value, set_data->pulse);

        uart_buff1[0] = set_data->type;
        uart_buff1[1] = ((p_meta->src.value >> 8) & 0xFF);
        uart_buff1[2] = (p_meta->src.value & 0xFF);
        uart_buff1[3] = set_data->pulse;
        uart_len1 = 4;
    }
    else if (set_data->type == RTLS_PRESSURE_TYPE)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Coming data: type - 0x%x,  from: 0x%x, 0x%x|0x%x complete.\n", 
                                            set_data->type, p_meta->src.value, set_data->pressure.pressure_up, 
                                            set_data->pressure.pressure_down);

        uart_buff1[0] = set_data->type;
        uart_buff1[1] = ((p_meta->src.value >> 8) & 0xFF);
        uart_buff1[2] = (p_meta->src.value & 0xFF);
        uart_buff1[3] = set_data->pressure.pressure_up;
        uart_buff1[4] = set_data->pressure.pressure_down;
        uart_len1 = 5;
    }
    else if (set_data->type == RTLS_RSSI_TYPE)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Coming data: type - 0x%x from: 0x%x, rssi = 0x%x addr = 0x%x complete.\n", 
                                            set_data->type, p_meta->src.value, set_data->rssi.rssi,
                                            set_data->rssi.tag_id);

        uart_buff1[0] = set_data->type;
        uart_buff1[1] = ((p_meta->src.value >> 8) & 0xFF);
        uart_buff1[2] = (p_meta->src.value & 0xFF);
        uart_buff1[3] = set_data->rssi.rssi;
        uart_buff1[4] = ((set_data->rssi.tag_id >> 8) & 0xFF);
        uart_buff1[5] = (set_data->rssi.tag_id & 0xFF);
        uart_len1 = 6;
    }
    else
    {
        NRF_MESH_ASSERT(0);
    }
    nrf_mesh_serial_tx(uart_buff1, uart_len1);
}

static void app_rtls_control_client_status_cb(const rtls_control_client_t * p_self, const access_message_rx_meta_t * p_meta,
                                                const rtls_control_status_params_t * p_in)
{
    if (p_in->type == RTLS_UUID_TYPE)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Coming data: type - 0x%x from: 0x%x, addr = 0x%x complete.\n", 
                                            p_in->type, p_meta->src.value,
                                            p_in->uuid.tag_id);

        /*uart_buff2[0] = p_in->type;
        uart_buff2[1] = ((p_meta->src.value >> 8) & 0xFF);
        uart_buff2[2] = (p_meta->src.value & 0xFF);
        uart_buff2[3] = ((p_in->uuid.tag_id >> 8) & 0xFF);
        uart_buff2[4] = (p_in->uuid.tag_id & 0xFF);
        uart_len2 = 5;*/
    }
    else
    {
        NRF_MESH_ASSERT(0);
    }
}

static void app_rtls_control_client_transaction_status_cb(access_model_handle_t model_handle, void * p_args,
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

/*
static void app_rtls_state_status_cb_t(const app_rtls_server_t * p_app, const rtls_set_params_t * set_data,
                                                                const access_message_rx_meta_t * p_meta)
{

}*/

static void app_control_model_init(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");

    for (uint32_t i = 0; i < 1; ++i)
    {
        m_control_clients[i].settings.p_callbacks = &client_cbs;
        m_control_clients[i].settings.timeout = 0;
        m_control_clients[i].settings.force_segmented = APP_FORCE_SEGMENTATION;
        m_control_clients[i].settings.transmic_size = APP_MIC_SIZE;

        ERROR_CHECK(rtls_control_client_init(&m_control_clients[i], i));
    }

}

static void app_model_init(void)
{
    /* Instantiate onoff server on element index APP_ONOFF_ELEMENT_INDEX */
    ERROR_CHECK(app_rtls_init(&m_rtls_server_0, APP_RTLS_ELEMENT_INDEX));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App RTLS Model Handle: %d\n", m_rtls_server_0.server.model_handle);
}

/*************************************************************************************************/

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
    "\t\t-------------------------------------------------------------------------------\n";
#endif

static void button_event_handler(uint32_t button_number)
{
    /* Increase button number because the buttons on the board is marked with 1 to 4 */
    button_number++;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);
    switch (button_number)
    {
        /* Pressing SW1 on the Development Kit will result in LED state to toggle and trigger
        the STATUS message to inform client about the state change. This is a demonstration of
        state change publication due to local event. */
        case 1:
            ERROR_CHECK(rtls_control_client_get(&m_control_clients[0]));
            break;

        /* Initiate node reset */
        case 4:
        {
            /* Clear all the states to reset the node. */
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
            break;
        }

        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
            break;
    }
}

static void app_rtt_input_handler(int key)
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

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    app_model_init();
    app_control_model_init();
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
    ERROR_CHECK(nrf_mesh_serial_init(NULL));
}

static void initialize(void)
{
    //__LOG_INIT(0, 0, LOG_CALLBACK_DEFAULT);
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS | LOG_SRC_BEARER, LOG_LEVEL_INFO | LOG_LEVEL_DBG1, LOG_CALLBACK_DEFAULT);
    //__LOG_INIT(LOG_SRC_APP | LOG_SRC_FRIEND, LOG_LEVEL_DBG1, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Server Demo -----\n");

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
    rtt_input_enable(app_rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

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
            .p_device_uri = EX_URI_RTLS_PC
            //.p_device_uri = EX_URI_LS_SERVER
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
    else
    {
        unicast_address_print();
    }

    ERROR_CHECK(nrf_mesh_serial_enable());
    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);

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
