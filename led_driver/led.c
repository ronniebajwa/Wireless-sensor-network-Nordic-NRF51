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

#include <stdint.h>
#include "nrf.h"
#include "boards.h"
#include "nrf_gpio.h"
#include "led.h"


/**@brief Function for initiating gpio and set LED in initial state.
 *
 */
void led_init(void)
{
    nrf_gpio_cfg_output(LED_RGB_RED);
    nrf_gpio_cfg_output(LED_RGB_GREEN);
    nrf_gpio_cfg_output(LED_RGB_BLUE);
    led_setState(LED_INIT_STATE);
}


/**@brief Function to set LED's colour.
 *
 * @param[in] colour LED's colour to set.
 */
void led_setState(const led_color_e color)
{
    if (color & LED_RED)
        nrf_gpio_pin_clear(LED_RGB_RED);
    else
        nrf_gpio_pin_set(LED_RGB_RED);

    if (color & LED_BLUE)
        nrf_gpio_pin_clear(LED_RGB_BLUE);
    else
        nrf_gpio_pin_set(LED_RGB_BLUE);

    if (color & LED_GREEN)
        nrf_gpio_pin_clear(LED_RGB_GREEN);
    else
        nrf_gpio_pin_set(LED_RGB_GREEN);
}


/**
 * @}
 */
