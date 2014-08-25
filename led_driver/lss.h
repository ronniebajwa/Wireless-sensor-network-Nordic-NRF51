/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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

/** @cond To make doxygen skip this file */

/** @file
 *
 * @defgroup ble_sdk_app_led_driver lss.h
 * @{
 * @ingroup ble_sdk_app_led_driver
 * @brief LED State Service for the LED Driver example application
 *
 */

#ifndef BLE_LSS_H__
#define BLE_LSS_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "led.h"

#define DEVICE_NAME               "LED_Driver"   /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME         "Nordic"       /**< Manufacturer. Will be passed to Device Information Service. */
#define DEFAULT_ADV_INTERVAL      100            /**< The advertising interval (in units of 0.625 ms. This value corresponds to 62.5 ms). */
#define DEFAULT_ADV_TIMEOUT       0              /**< The advertising timeout in units of seconds. 0 means infinity. */
 
#define MY_UUID_LED_SERVICE       0x1866         /**< Custom LED State Service UUID */
#define MY_UUID_LED_STATE_CHAR    0x2A57         /**< Custom LED State Characteristic UUID */
#define MY_UUID_ADV_INTERVAL_CHAR 0x2A58         /**< Custom Advertising Interval Characteristic UUID */

typedef struct
{
    led_color_e state;
    uint16_t    adv_interval;
} ble_device_params_t;

/**@brief LED State Service event type. */
typedef enum
{
    BLE_LSS_EVT_NOTIFICATION_ENABLED, /**< LED State value notification enabled event. */
    BLE_LSS_EVT_NOTIFICATION_DISABLED /**< LED State value notification disabled event. */
} ble_lss_evt_type_t;

/**@brief LED State Service event. */
typedef struct
{
    ble_lss_evt_type_t evt_type; /**< Type of event. */
} ble_lss_evt_t;

// Forward declaration of the ble_lss_t type.
typedef struct ble_lss_s ble_lss_t;

/**@brief LED State Service event handler type. */
typedef void (* ble_lss_evt_handler_t) (ble_lss_t * p_lss, ble_lss_evt_t * p_evt);

/**@brief LED State Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    ble_lss_evt_handler_t        evt_handler; /**< Event handler to be called for handling events in the LED State Service. */
    ble_srv_cccd_security_mode_t lss_attr_md; /**< Initial security level for led state service measurement attribute */
    ble_device_params_t          dev_params;  /**< Service characteristics value */
} ble_lss_init_t;

/**@brief LED State Service structure. This contains various status information for the service. */
typedef struct ble_lss_s
{
    ble_lss_evt_handler_t    evt_handler;    /**< Event handler to be called for handling events in the LED State Service. */
    uint16_t                 service_handle; /**< Handle of LED State Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t lss_ls_handle;  /**< Handles related to the LED state characteristic. */
    ble_gatts_char_handles_t lss_ai_handle;  /**< Handles related to the advertising interval characteristic. */
    ble_device_params_t      dev_params;     /**< Service characteristics value */
    uint16_t                 conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
} ble_lss_t;

/****************************************************************
* NON-STATIC FUNCTIONS DECLARATIONS
****************************************************************/

/**@brief Function for initializing the LED State Service.
 *
 * @param[out]  p_lss       LED State Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_lss_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_lss_init(ble_lss_t * p_lss, const ble_lss_init_t * p_lss_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the LED State Service.
 *
 * @param[in]   p_lss      LED State Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_lss_on_ble_evt(ble_lss_t * p_lss, ble_evt_t * p_ble_evt);

#endif // BLE_LSS_H__

/** @} */
