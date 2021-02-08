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

#ifndef NRF_MESH_CONFIG_APP_H__
#define NRF_MESH_CONFIG_APP_H__


//-----------------------------light_switch_example_common.h-----------------------
/** Maximum available number of servers to choose correct TTL for the network. */
#define MAX_AVAILABLE_SERVER_NODE_NUMBER  (40)

/** Number of On-Off client models on the Switch Node */
#define CLIENT_MODEL_INSTANCE_COUNT  (2)

//-----------------------------nrf_mesh_config_app.h---------------------------

/**
 * @defgroup NRF_MESH_CONFIG_APP nRF Mesh app config
 *
 * Application side configuration file. Should be copied into every
 * application, and customized to fit its requirements.
 * @{
 */

/**
 * @defgroup MODEL_CONFIG Model layer configuration parameters
 */

/** Define for acknowledging message transaction timeout.
 * @note @tagMeshSp recommends this to be minimum 60s. However, using
 * recommendation can result in client getting blocked for a significant amount of time (60s), if
 * acknowledged transaction does not receive a response.
 */
#define MODEL_ACKNOWLEDGED_TRANSACTION_TIMEOUT  (SEC_TO_US(10))

/** @} end of MODEL_CONFIG */

/**
 * @defgroup DEVICE_CONFIG Device configuration
 *
 * @{
 */

/** Device company identifier. */
#define DEVICE_COMPANY_ID (ACCESS_COMPANY_ID_NORDIC)

/** Device product identifier. */
#define DEVICE_PRODUCT_ID (0x0000)

/** Device version identifier. */
#define DEVICE_VERSION_ID (0x0000)

/** @} end of DEVICE_CONFIG */

/**
 * @defgroup ACCESS_CONFIG Access layer configuration
 * @{
 */

/**
 * The default TTL value for the node.
 */
#define ACCESS_DEFAULT_TTL (MAX_AVAILABLE_SERVER_NODE_NUMBER > NRF_MESH_TTL_MAX ? \
        NRF_MESH_TTL_MAX : MAX_AVAILABLE_SERVER_NODE_NUMBER)

/**
 * The number of models in the application.
 *
 * @note To fit the configuration and health models, this value must equal at least
 * the number of models needed by the application plus two.
 */
#define ACCESS_MODEL_COUNT (1 + /* Configuration server */  \
                            1 + /* Health server */  \
                            1   /* Generic OnOff client (2 groups) */)

/**
 * The number of elements in the application.
 *
 * @warning If the application is to support _multiple instances_ of the _same_ model, these instances
 * cannot be in the same element and a separate element is needed for each new instance of the same model.
 */
#define ACCESS_ELEMENT_COUNT (1) /* One element per Generic OnOff client instance */

/**
 * The number of allocated subscription lists for the application.
 *
 * @note This value must equal @ref ACCESS_MODEL_COUNT minus the number of
 * models operating on shared states.
 */
#define ACCESS_SUBSCRIPTION_LIST_COUNT (ACCESS_MODEL_COUNT)

/**
 * @defgroup ACCESS_RELIABLE_CONFIG Configuration of access layer reliable transfer
 * @{
 */

/** Number of the allowed parallel transfers (size of the internal context pool). */
#define ACCESS_RELIABLE_TRANSFER_COUNT (ACCESS_MODEL_COUNT)

/** @} end of ACCESS_RELIABLE_CONFIG */


/** @} end of ACCESS_CONFIG */


/**
 * @ingroup HEALTH_MODEL
 * @{
 */

/** The number of instances of the health server model. */
#define HEALTH_SERVER_ELEMENT_COUNT (1)

/** @} end of HEALTH_MODEL */


/**
 * @defgroup DSM_CONFIG Device State Manager configuration
 * Sizes for the internal storage of the Device State Manager.
 * @{
 */
/** Maximum number of subnetworks. */
#define DSM_SUBNET_MAX                                  (4)
/** Maximum number of applications. */
#define DSM_APP_MAX                                     (8)
/** Maximum number of device keys. */
#define DSM_DEVICE_MAX                                  (1)
/** Maximum number of virtual addresses. */
#define DSM_VIRTUAL_ADDR_MAX                            (1)
/** Maximum number of non-virtual addresses. One for each of the servers and a group address. */
#define DSM_NONVIRTUAL_ADDR_MAX                         (ACCESS_MODEL_COUNT + 1)
/** @} end of DSM_CONFIG */

/** @} */

/**
 * @defgroup NRF_MESH_CONFIG_CORE Compile time configuration
 * Configuration of the compilation of the core mesh modules.
 * @ingroup CORE_CONFIG
 * @{
 */

/**
 * @defgroup MESH_CONFIG_GATT GATT configuration defines
 * @{
 */
/** PB-GATT feature. To be enabled only in combination with linking GATT files. */
#define MESH_FEATURE_PB_GATT_ENABLED                    (1)
/** GATT proxy feature. To be enabled only in combination with linking GATT proxy files. */
#define MESH_FEATURE_GATT_PROXY_ENABLED                 (1)
/** @} end of MESH_CONFIG_GATT */

/**
 * @defgroup BLE_SOFTDEVICE_SUPPORT_CONFIG BLE SoftDevice support module configuration.
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Configuration for compile time. Part of BLE SoftDevice support module.
 *
 * @{
 */
/** GAP device name. */
#define GAP_DEVICE_NAME                 "nRF5x Mesh Switch"
/** @} end of BLE_SOFTDEVICE_SUPPORT_CONFIG */

//-----------------------------------------example_common.h-----------

#define RTT_INPUT_POLL_PERIOD_MS    (100)

#define LED_BLINK_INTERVAL_MS           (200)
#define LED_BLINK_SHORT_INTERVAL_MS     (50)
#define LED_BLINK_CNT_START             (2)
#define LED_BLINK_CNT_RESET             (3)
#define LED_BLINK_CNT_PROV              (4)
#define LED_BLINK_CNT_NO_REPLY          (6)
#define LED_BLINK_CNT_ERROR             (6)

/* An interval larger than half a second might not show LED blinking effect. */
#define LED_BLINK_ATTENTION_INTERVAL_MS (50)
#define LED_BLINK_ATTENTION_COUNT(s)    (((s) * 500) / LED_BLINK_ATTENTION_INTERVAL_MS)

/**
 * Clock configuration for Nordic development boards.
 */
#if defined(S110)
    #define DEV_BOARD_LF_CLK_CFG  NRF_CLOCK_LFCLKSRC_XTAL_20_PPM
#elif NRF_SD_BLE_API_VERSION >= 5
    #define DEV_BOARD_LF_CLK_CFG  { \
        .source = NRF_SDH_CLOCK_LF_SRC, \
        .rc_ctiv = NRF_SDH_CLOCK_LF_RC_CTIV, \
        .rc_temp_ctiv = NRF_SDH_CLOCK_LF_RC_TEMP_CTIV, \
        .accuracy = NRF_SDH_CLOCK_LF_ACCURACY \
    }
#else
    #define DEV_BOARD_LF_CLK_CFG  { \
        .source = NRF_CLOCK_LF_SRC_XTAL, \
        .rc_ctiv = 0, \
        .rc_temp_ctiv = 0, \
        .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM \
    }
#endif

/** Uniform Resource Identifiers (URIs) for the examples.
 *
 *
 * @note Replace the example URI strings with the desired URIs for the end products. The URI strings
 * should be coded as specified in the Bluetooth Core Specification Supplement v6, section 1.18.
 */
#define EX_URI_BEACON        URI_SCHEME_EXAMPLE "URI for Beacon example"
#define EX_URI_DFU           URI_SCHEME_EXAMPLE "URI for DFU example"
#define EX_URI_ENOCEAN       URI_SCHEME_EXAMPLE "URI for Enocean example"
#define EX_URI_DM_CLIENT     URI_SCHEME_EXAMPLE "URI for Dimming Client example"
#define EX_URI_DM_SERVER     URI_SCHEME_EXAMPLE "URI for Dimming Server example"
#define EX_URI_LPN           URI_SCHEME_EXAMPLE "URI for LPN example"
#define EX_URI_LS_CLIENT     URI_SCHEME_EXAMPLE "URI for LS Client example"
#define EX_URI_LS_SERVER     URI_SCHEME_EXAMPLE "URI for LS Server example"
#define EX_URI_LL_CLIENT     URI_SCHEME_EXAMPLE "URI for Light Lightness Client example"
#define EX_URI_LL_SERVER     URI_SCHEME_EXAMPLE "URI for Light Lightness Setup Server example"
#define EX_URI_LC_SERVER     URI_SCHEME_EXAMPLE "URI for Light LC Setup Server example"
#define EX_URI_CTL_CLIENT    URI_SCHEME_EXAMPLE "URI for Light CTL Client example"
#define EX_URI_CTL_SERVER    URI_SCHEME_EXAMPLE "URI for Light CTL Setup Server example"
#define EX_URI_CTL_LC_SERVER URI_SCHEME_EXAMPLE "URI for Light CTL+LC Setup Servers example"
#define EX_URI_PBR_CLIENT    URI_SCHEME_EXAMPLE "URI for PB Remote Client example"
#define EX_URI_PBR_SERVER    URI_SCHEME_EXAMPLE "URI for PB Remote Server example"
#define EX_URI_SERIAL        URI_SCHEME_EXAMPLE "URI for Serial example"
#define EX_URI_SENSOR_SERVER URI_SCHEME_EXAMPLE "URI for Sensor Server example"
#define EX_URI_SENSOR_CLIENT URI_SCHEME_EXAMPLE "URI for Sensor Client example"

/** Static authentication data. */
#define STATIC_AUTH_DATA {0x6E, 0x6F, 0x72, 0x64, 0x69, 0x63, 0x5F, 0x65, 0x78, 0x61, 0x6D, 0x70, 0x6C, 0x65, 0x5F, 0x31}

/** Static authentication data for remote provisioning example. */
#define STATIC_AUTH_DATA_PB_REMOTE {0xc7, 0xf7, 0x9b, 0xec, 0x9c, 0xf9, 0x74, 0xdd, 0xb9, 0x62, 0xbd, 0x9f, 0xd1, 0x72, 0xdd, 0x73}

//-----------------------------------------nrf_mesh_config_examples.h-----------

/**
 * @defgroup NRF_MESH_CONFIG_EXAMPLES Application support module configuration
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Configuration of the application support modules.
 * @{
 */

/**
 * @defgroup RTT_INPUT_CONFIG RTT Input module configuration (mesh examples)
 * Configuration for compile time. Part of the RTT Input module for mesh examples.
 * @{
 */

/** Enable RTT Input support. */
#ifndef RTT_INPUT_ENABLED
#define RTT_INPUT_ENABLED 1
#endif

/** @} end of RTT_INPUT_CONFIG */

/**
 * @defgroup SIMPLE_HAL_CONFIG Simple Hardware Abstraction Layer configuration (mesh examples)
 * Configuration for compile time. Part of the Simple Hardware Abstraction Layer for mesh examples.
 * @{
 */

/** Enable support for LEDs. */
#ifndef SIMPLE_HAL_LEDS_ENABLED
#define SIMPLE_HAL_LEDS_ENABLED 1
#endif

/** @} end of SIMPLE_HAL_CONFIG */

/**
 * @defgroup BLE_SOFTDEVICE_SUPPORT_CONFIG BLE SoftDevice support module configuration
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Configuration for compile time. Part of the BLE SoftDevice support module.
 *
 * @{
 */

/** GAP device name. */
#ifndef GAP_DEVICE_NAME
#define GAP_DEVICE_NAME                 "nRF5x Mesh Node"
#endif

/** Minimum acceptable connection interval. */
#ifndef MIN_CONN_INTERVAL
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(150,  UNIT_1_25_MS)
#endif

/** Maximum acceptable connection interval. */
#ifndef MAX_CONN_INTERVAL
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(250,  UNIT_1_25_MS)
#endif

/** Slave latency. */
#ifndef SLAVE_LATENCY
#define SLAVE_LATENCY                   0
#endif

/** Connection supervisory timeout (4 seconds). */
#ifndef CONN_SUP_TIMEOUT
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)
#endif

/** Time from initiating the event (connect or start of notification) to the first
 * time sd_ble_gap_conn_param_update is called. */
#ifndef FIRST_CONN_PARAMS_UPDATE_DELAY
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(100)
#endif

/** Time between each call to sd_ble_gap_conn_param_update after the first call. */
#ifndef NEXT_CONN_PARAMS_UPDATE_DELAY
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(2000)
#endif

/** Number of attempts before giving up the connection parameter negotiation. */
#ifndef MAX_CONN_PARAMS_UPDATE_COUNT
#define MAX_CONN_PARAMS_UPDATE_COUNT    3
#endif

/** @} end of BLE_SOFTDEVICE_SUPPORT_CONFIG */

/**
 * @defgroup DFU_SUPPORT_CONFIG BLE DFU support module configuration
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Configuration for compile time. Part of the BLE DFU support module.
 *
 * @{
 */

/** Enable BLE DFU support module. */
#ifndef BLE_DFU_SUPPORT_ENABLED
#define BLE_DFU_SUPPORT_ENABLED 0
#endif

/** @} end of DFU_SUPPORT_CONFIG */

/** @} end of NRF_MESH_CONFIG_EXAMPLES */









/** @} end of NRF_MESH_CONFIG_CORE */

#endif /* NRF_MESH_CONFIG_APP_H__ */
