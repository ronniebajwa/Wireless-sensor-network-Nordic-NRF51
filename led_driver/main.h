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
 * @defgroup ble_sdk_app_led_driver_main Main header file
 * @{
 * @ingroup ble_sdk_app_led_driver
 * @brief Main source file prototypes
 *
 */

#ifndef MAIN_H__
#define MAIN_H__

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                   /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/
#define APP_TIMER_PRESCALER             0                   /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            3                   /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         5                   /**< Size of timer operation queues. */
#define UPDATE_PERIOD                   2000                /**< How often advertisement packet is updated (in ms) */
#define APP_GPIOTE_MAX_USERS            1                   /**< Maximum number of users of the GPIOTE handler. */
#define DEAD_BEEF                       0xDEADBEEF          /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define SERVICES_COUNT                  1                   /**< Number of services which send data through send response packets. */

/**********************************************************
* Connections parameters
**********************************************************/

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Maximum acceptable interval without packet sent(4 second). */
   
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

#endif // MAIN_H__

/** @} */
/** @endcond */
