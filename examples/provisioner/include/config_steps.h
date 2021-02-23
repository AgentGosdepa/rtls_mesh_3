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

CFG_DECLARE(NODE_SETUP_IDLE,                                       ACCESS_HANDLE_INVALID)
CFG_DECLARE(NODE_SETUP_GET_NEXT_ELEMENT,                           ACCESS_HANDLE_INVALID)
CFG_DECLARE(NODE_SETUP_CONFIG_COMPOSITION_GET,                     ACCESS_HANDLE_INVALID)
CFG_DECLARE(NODE_SETUP_CONFIG_NETWORK_TRANSMIT,                    ACCESS_HANDLE_INVALID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_ADD,                          ACCESS_HANDLE_INVALID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_HEALTH,                  HEALTH_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_ONOFF_SERVER,            GENERIC_ONOFF_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_ONOFF_CLIENT,            GENERIC_ONOFF_CLIENT_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_LEVEL_SERVER,            GENERIC_LEVEL_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_LEVEL_CLIENT,            GENERIC_LEVEL_CLIENT_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_LL_CLIENT,               LIGHT_LIGHTNESS_CLIENT_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_LL_SERVER,               LIGHT_LIGHTNESS_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_LL_SETUP_SERVER,         LIGHT_LIGHTNESS_SETUP_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_PONOFF_SERVER,           GENERIC_PONOFF_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_PONOFF_SETUP_SERVER,     GENERIC_PONOFF_SETUP_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_DTT_SERVER,              GENERIC_DTT_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_LC_SERVER,               LIGHT_LC_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_LC_SETUP_SERVER,         LIGHT_LC_SETUP_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_CTL_SERVER,              LIGHT_CTL_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_CTL_SETUP_SERVER,        LIGHT_CTL_SETUP_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_CTL_TEMPERATURE_SERVER,  LIGHT_CTL_TEMPERATURE_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_CTL_CLIENT,              LIGHT_CTL_CLIENT_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_SENSOR_CLIENT,           SENSOR_CLIENT_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_SENSOR_SERVER,           SENSOR_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_SENSOR_SETUP_SERVER,     SENSOR_SETUP_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_HEALTH,                  HEALTH_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_ONOFF_SERVER,            GENERIC_ONOFF_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_ONOFF_CLIENT,            GENERIC_ONOFF_CLIENT_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_ONOFF_SERVER,           GENERIC_ONOFF_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_ONOFF_CLIENT,           GENERIC_ONOFF_CLIENT_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_LEVEL_SERVER,            GENERIC_LEVEL_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_LEVEL_CLIENT,            GENERIC_LEVEL_CLIENT_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_LEVEL_SERVER,           GENERIC_LEVEL_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_LEVEL_CLIENT,           GENERIC_LEVEL_CLIENT_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_LL_SERVER,               LIGHT_LIGHTNESS_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_LL_CLIENT,               LIGHT_LIGHTNESS_CLIENT_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_LL_SERVER,              LIGHT_LIGHTNESS_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_LL_CLIENT,              LIGHT_LIGHTNESS_CLIENT_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_LL_SETUP_SERVER,         LIGHT_LIGHTNESS_SETUP_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_LL_SETUP_SERVER,        LIGHT_LIGHTNESS_SETUP_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_PONOFF_SERVER,           GENERIC_PONOFF_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_PONOFF_SERVER,          GENERIC_PONOFF_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_PONOFF_SETUP_SERVER,     GENERIC_PONOFF_SETUP_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_PONOFF_SETUP_SERVER,    GENERIC_PONOFF_SETUP_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_DTT_SERVER,              GENERIC_DTT_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_DTT_SERVER,             GENERIC_DTT_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_LC_SERVER,               LIGHT_LC_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_LC_SERVER,              LIGHT_LC_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_LC_SERVER_ON_SENSOR_STATUS, LIGHT_LC_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_LC_SETUP_SERVER,         LIGHT_LC_SETUP_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_LC_SETUP_SERVER,        LIGHT_LC_SETUP_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_CTL_SERVER,              LIGHT_CTL_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_CTL_SERVER,             LIGHT_CTL_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_CTL_SETUP_SERVER,        LIGHT_CTL_SETUP_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_CTL_SETUP_SERVER,       LIGHT_CTL_SETUP_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_CTL_TEMPERATURE_SERVER,  LIGHT_CTL_TEMPERATURE_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_CTL_TEMPERATURE_SERVER, LIGHT_CTL_TEMPERATURE_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_CTL_CLIENT,              LIGHT_CTL_CLIENT_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_CTL_CLIENT,             LIGHT_CTL_CLIENT_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_SENSOR_SERVER,           SENSOR_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_SENSOR_SERVER,          SENSOR_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_SENSOR_SETUP_SERVER,     SENSOR_SETUP_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_SENSOR_SETUP_SERVER,    SENSOR_SETUP_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_SENSOR_CLIENT,           SENSOR_CLIENT_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_SENSOR_CLIENT,          SENSOR_CLIENT_MODEL_ID)
CFG_DECLARE(NODE_SETUP_DONE,                                       ACCESS_HANDLE_INVALID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_RTLS_MODEL_CLIENT,		RTLS_CLIENT_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_RTLS_MODEL_CLIENT,        RTLS_CLIENT_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_RTLS_MODEL_SERVER,    	RTLS_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_RTLS_MODEL_SERVER,       RTLS_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_RTLS_RSSI_CLIENT,    		RTLS_RSSI_CLIENT_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_PUBLICATION_RTLS_RSSI_CLIENT,         RTLS_RSSI_CLIENT_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_APPKEY_BIND_RTLS_RSSI_SERVER,    		RTLS_RSSI_SERVER_MODEL_ID)
CFG_DECLARE(NODE_SETUP_CONFIG_SUBSCRIPTION_RTLS_RSSI_SERVER,        RTLS_RSSI_SERVER_MODEL_ID)