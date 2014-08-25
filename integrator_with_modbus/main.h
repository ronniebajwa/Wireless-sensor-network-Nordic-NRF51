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
 * @defgroup ble_sdk_app_data_integrator_with_modbus_main main.h
 * @{
 * @ingroup ble_sdk_app_integrator_with_modbus
 * @brief Main file for integrator_with_modbus application for nRF51822 evaluation board. Whole example project consist of integrator_with_modbus and
 *        led_driver.
 *
 */

#ifndef MAIN_H__
#define MAIN_H__

#define BUTTON_CLEAN_LIST      BUTTON_1                                 /**< Button used to clearing sensors list */
#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50, APP_TIMER_PRESCALER) /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define LED_RQ_PENDING LED_0 /**< Change state when advertising packet from proper device is received */
#define LED_CONNECTED  LED_1 /**< Is on when device is connected. */

#define SEC_PARAM_BOND            1                    /**< Perform bonding. */
#define SEC_PARAM_MITM            1                    /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE /**< No I/O capabilities. */
#define SEC_PARAM_OOB             0                    /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE    7                    /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE    16                   /**< Maximum encryption key size. */

#define APP_GPIOTE_MAX_USERS 1 /**< Maximum number of users of the GPIOTE handler. */

#define APP_TIMER_PRESCALER     0 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS    2 /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE 5

#define SCHED_MAX_EVENT_DATA_SIZE 0 /**< Maximum data size if event message. */
#define SCHED_QUEUE_SIZE          5 /**< Maximum number of events in scheduler queue. */

#endif // MAIN_H__

/** @} */
/** @endcond */
