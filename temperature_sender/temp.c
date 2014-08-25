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

#include "softdevice_handler.h"
#include "temp.h"

/**@brief    Function for getting SoC temperature via SoftDevice API.
 * @details  This function will fetch the SoC's temperature result from the ADC.
 *
 * @param[out] temp Temperature value to return.
 *
 * @retval NRF_SUCCESS if measurement were taken properly, 
 * @retval NRF_ERROR_SOC_RAND_NOT_ENOUGH_VALUES when no bytes were written
 * to the buffer, because there were not enough bytes available.
 */
uint32_t take_temperature(uint32_t * temp)
{
    uint32_t err_code = sd_temp_get((int32_t *)temp);

    *temp = TEMP_IN_CELSIUS(*temp);

    return err_code;
}


/**
 * @}
 */
