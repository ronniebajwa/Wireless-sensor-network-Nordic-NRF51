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

/** @file
 *
 * @defgroup ble_sdk_app_temp_sender_battery
 * @{
 * @ingroup ble_sdk_app_temp_sender
 * @brief Battery Level Hardware Handling prototypes
 *
 */

#ifndef BATTERY_H__
#define BATTERY_H__

#include <stdint.h>

#define FAULTY_BATT_LEVEL              255   /**< Value to indicate improper measurement */
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS  1200  /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION   3     /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS 270   /**< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com. */

/**@brief Function configures adc in order to measure battery level.
 */
void battery_init(void);

/**@brief   Function for getting battery level using ADC.
 * @details This function also converts adc register value to percentage level.
 *
 * @param[out] bl Battery level in percentage.
 *
 * @retval NRF_SUCCES is measurement were properly taken, NRF_ERROR_BUSY otherwise.
 */
uint32_t take_battery_level(uint8_t * bl);

#endif // BATTERY_H__

/** @} */
/** @endcond */
