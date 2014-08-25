/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_integrator_with_modbus_devices devices.h
 * @{
 * @ingroup ble_sdk_app_integrator_with_modbus
 * @brief Header for for sensors management
 *
 */

#ifndef __TEMP_H
#define __TEMP_H

#include <inttypes.h>
#include "app_util.h"
#include "softdevice_handler.h"

#define FAULTY_TEMPERATURE   1000 /**< Incidates faulty battery level measurement. */
#define FAULTY_BATTERY_LEVEL 0xFF /**< Incidates faulty temperature measurement. */

#define SCAN_INTERVAL 0x00A0      /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW   0x00A0      /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT  0x0001      /**< Determines scan timeout in seconds. */

#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(7.5, UNIT_1_25_MS) /**< Determines maximum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(30, UNIT_1_25_MS)  /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY           0                                /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT     MSEC_TO_UNITS(4000, UNIT_10_MS)  /**< Determines supervision time-out in units of 10 millisecond. */

typedef enum
{
    BLE_NO_SCAN,        /**< No advertising running. */
    BLE_WHITELIST_SCAN, /**< Advertising with whitelist. */
    BLE_FAST_SCAN,      /**< Fast advertising running. */
} ble_advertising_mode_t;

void bt_scan_start(void);

/***************************************************************
 * Advertising packets processing
 ****************************************************************/

/**
 * @brief Handle advertising packet to find out if it is packet from a compatible temperature sensor. If so,
 *        retrieve useful data from packet, add device to list and eventually update modbus registers.
 *
 * @param[in] address address Device address.
 * @param[in] data data       Advertising packet data.
 *
 * @retval NRF_SUCCESS if the data type is found in the report, NRF_ERROR_NOT_FOUND otherwise.
 */
uint8_t bt_handle_temp_sensor(ble_gap_addr_t * address, uint8_array_t * data);


/**
 * @brief Handle advertising packet to find out if it is packet from a compatible led_driver. If so,
 *        retrieve useful data from packet, add device to list and eventually update modbus registers.
 *
 * @param[in] address address Device address.
 * @param[in] data data       Advertising packet data.
 *
 * @retval NRF_SUCCESS if the data type is found in the report, NRF_ERROR_NOT_FOUND otherwise.
 */
uint8_t bt_handle_led_driver(ble_gap_addr_t * address, uint8_array_t * data);

/***************************************************************
 * Bluetooth connectivity handling
 ****************************************************************/


/**
 * @brief Connect to a device.
 *
 * @retval NRF_SUCCESS if connection was sucessfully initiated, error code otherwise.
 */
uint32_t bt_connect(void);

/**
 * @brief Update attribute value.
 *
 * @param[in] conn_handle Connection handle.
 *
 * @retval NRF_SUCCESS if operation was successful, error code otherwise.
 */
uint32_t bt_write_to_device(const uint16_t conn_handle);


/**
 * @brief Disconnect from peer.
 *
 * @param[in] conn_handle Connection handle.
 *
 * @retval NRF_SUCCESS if disconnection was sucessfully initiated, error code otherwise.
 */
uint32_t bt_disconnect(const uint16_t conn_handle);


/**
 * @brief Clean devices list.
 *
 */
void bt_clean_list(void);

#endif // TEMP_H__
