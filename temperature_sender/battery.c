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

#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "main.h"
#include "battery.h"
#include "app_util.h"

/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in] ADC_VALUE ADC result.
 *
 * @retval    Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
    ((((ADC_VALUE) *ADC_REF_VOLTAGE_IN_MILLIVOLTS) / 255) * ADC_PRE_SCALING_COMPENSATION)


/**@brief   Function for getting battery level using ADC.
 * @details This function also converts adc register value to percentage level.
 *
 * @param[out] bl Battery level in percentage.
 *
 * @retval NRF_SUCCES is measurement were properly taken, NRF_ERROR_BUSY otherwise.
 */
uint32_t take_battery_level(uint8_t * bl)
{
    uint32_t err_code = NRF_ERROR_BUSY;

    if (NRF_ADC->EVENTS_END != 0)
    {
        uint8_t  adc_result;
        uint16_t batt_lvl_in_milli_volts;

        NRF_ADC->EVENTS_END = 0;
        adc_result          = NRF_ADC->RESULT;
        NRF_ADC->TASKS_STOP = 1;

        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;
                                  
        *bl = battery_level_in_percent(batt_lvl_in_milli_volts);

        NRF_ADC->TASKS_START = 1;
        err_code             = NRF_SUCCESS;
    }

    return err_code;
}


/**@brief Function configures adc in order to measure battery level.
 */
void battery_init(void)
{
    // Configure ADC
    NRF_ADC->INTENSET = ADC_INTENSET_END_Msk;
    NRF_ADC->CONFIG   = (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos) |
                        (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                        (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                        (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos) |
                        (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
    NRF_ADC->EVENTS_END = 0;
    NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Enabled;

    NRF_ADC->TASKS_START = 1;
}


/**
 * @}
 */
