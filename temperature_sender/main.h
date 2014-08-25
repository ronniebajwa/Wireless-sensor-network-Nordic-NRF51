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
 * @defgroup ble_sdk_app_temp_sender_main Main header file
 * @{
 * @ingroup ble_sdk_app_temp_sender
 * @brief Main source file prototypes
 *
 */

#ifndef MAIN_H__
#define MAIN_H__

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                   /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define DEVICE_NAME                     "Thermometer"       /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "Nordic"            /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                3000                /**< The advertising interval (in units of 0.625 ms. This value corresponds to 1,875s). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                   /**< The advertising timeout in units of seconds. */
                                       
#define APP_TIMER_PRESCALER             0                   /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            3                   /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         5                   /**< Size of timer operation queues. */
                                       
#define UPDATE_PERIOD                   1875                /**< How often advertisement packet is updated (in ms) */
#define APP_GPIOTE_MAX_USERS            1                   /**< Maximum number of users of the GPIOTE handler. */
                                       
#define DEAD_BEEF                       0xDEADBEEF          /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
                                       
#define SERVICES_COUNT                  2                   /**< Number of services which send data through send response packets. */

#endif // MAIN_H__

/** @} */
/** @endcond */
