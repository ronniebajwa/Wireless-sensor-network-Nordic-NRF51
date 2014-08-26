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
 * @defgroup ble_sdk_app_led_driver led.h
 * @{
 * @ingroup ble_sdk_app_led_driver
 * @brief LED control for the led_driver examplary application
 *
 */

#ifndef LED_H__
#define LED_H__

/**@brief Descibes diode colour
 */
typedef enum
{
    LED_OFF    = 0,
    LED_RED    = 1,
    LED_GREEN  = 2,
    LED_YELLOW = 3,
    LED_BLUE   = 4,
    LED_VIOLET = 5,
    LED_CYAN   = 6,
    LED_WHITE  = 7,
    LED_MAX
} led_color_e;

#define  LED_INIT_STATE LED_RED /**< Indicates initial LED state after calling led_init */

/**@brief Function for initiating gpio and set LED in initial state.
 *
 */
void led_init(void);

/**@brief Function to set LED's colour.
 *
 * @param[in] colour LED's colour to set.
 */
void led_setState(const led_color_e color);

#endif // LED_H__

/** @} */
/** @endcond */
