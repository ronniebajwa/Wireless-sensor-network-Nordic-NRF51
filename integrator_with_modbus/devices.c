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

#include "nrf_gpio.h"
#include "pstorage.h"
#include <string.h>
#include "devices.h"
#include "modbus.h"
#include "btble4.h"
#include "ble_hci.h"

extern bool m_memory_access_in_progress; /**< Flag to keep track of ongoing operations on persistent memory. */

extern mb_bt_memory_t g_memory;
bt_devices_list_t     g_devs_list;
ble_gap_scan_params_t g_scan_param; /**< Scan parameters requested for scanning and connection. */
int8_t                g_conn_dev_idx = -1;

extern uint8_t g_buffer[MAX_BUFFER_SIZE];

/**************************************************************
*   STATIC FUNCTIONS DECLARATIONS
**************************************************************/
static uint8_t parse_service_data(uint8_array_t * adv_data,
                                  uint16_t        uuid,
                                  uint8_t       * value,
                                  uint8_t         length);

/**
 * @brief Connection parameters requested for connection.
 */
const ble_gap_conn_params_t g_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL, // Minimum connection
    (uint16_t)MAX_CONNECTION_INTERVAL, // Maximum connection
    0,                                 // Slave latency
    (uint16_t)SUPERVISION_TIMEOUT      // Supervision time-out
};

/***************************************************************
 * Devices' list manipulating
 ****************************************************************/

/**
 * @brief Browses device's list to check if passed address is already saved on the list, if so, on which position.
 *
 * @param[in]  The device address.
 * @param[out]  Device position on the list.
 *
 * @retval Return 1 if device was found, and 0 if not
 */
static uint8_t find_in_list(ble_gap_addr_t * address, uint8_t * pos)
{
    *pos = 0;

    for (int i = 0; i < g_devs_list.number && i < BT_MAX_DEVICES; ++i)
    {
        if (memcmp(address->addr, (void *)g_devs_list.device[i].address.addr,
                   BLE_GAP_ADDR_LEN) == 0)
        {
            if (pos != NULL)
                *pos = i;
            return 1;
        }
    }
    return 0;
}


/**
 * @brief Clean device's list.
 *
 */
void bt_clean_list(void)
{
    g_devs_list.number = 0;
    g_memory.number    = 0;

    for (uint8_t i = 0; i < BT_MAX_DEVICES; ++i)
    {
        memset(&g_memory, 0, sizeof (g_memory));
    }
}


/**
 * @brief Add device to list
 *
 * @param[in]  The device address. The first free index is indicated by global integer.
 *
 */
static void add_to_list(ble_gap_addr_t * address, uint8_t type)
{
    memcpy((void *)g_devs_list.device[g_devs_list.number].address.addr,
           address->addr,
           BLE_GAP_ADDR_LEN);
    g_devs_list.device[g_devs_list.number].address.addr_type = address->addr_type;
    g_devs_list.device[g_devs_list.number].type              = type;
    ++g_devs_list.number;
    g_memory.number = g_devs_list.number;
}


/**@breif Function to start scanning.
 */
void bt_scan_start(void)
{
    uint32_t err_code;
    uint32_t count;

    // Verify if there is any flash access pending, if yes delay starting scanning until
    // it's complete.
    err_code = pstorage_access_status_get(&count);
    APP_ERROR_CHECK(err_code);

    if (count != 0)
    {
        m_memory_access_in_progress = true;
        return;
    }

    // No devices in whitelist, hence non selective performed.
    g_scan_param.active      = 0;             // Active scanning set.
    g_scan_param.selective   = 0;             // Selective scanning not set.
    g_scan_param.interval    = SCAN_INTERVAL; // Scan interval.
    g_scan_param.window      = SCAN_WINDOW;   // Scan window.
    g_scan_param.p_whitelist = NULL;          // No whitelist provided.
    g_scan_param.timeout     = SCAN_TIMEOUT;

    err_code = sd_ble_gap_scan_start(&g_scan_param);
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Browses advertising packet for useful data (temperature & battery level).
                    If whole this data is found, function check if the device's address is already on the list.
                    If the device isn't on the list, it is added, and sensors count is incremented
 *
 * @param[in]  The device address.
 * @param[in]  Advertising packet data.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
uint8_t bt_handle_temp_sensor(ble_gap_addr_t * address, uint8_array_t * data)
{
    uint32_t        err_code;
    uint32_t        temp;
    uint8_t         batt;
    uint8_t         pos;
    temp_sensor_t * temp_sensor;

    // err_code = parse_temp(data, &temp);
    err_code = parse_service_data(data, 0x1809, (uint8_t *)&temp, 4);

    if (err_code != NRF_SUCCESS)
        return NRF_ERROR_NOT_FOUND;

    // err_code = parse_batt_level(data, &batt);
    err_code = parse_service_data(data, 0x180f, &batt, 1);

    if (err_code != NRF_SUCCESS)
        return NRF_ERROR_NOT_FOUND;

    if (!find_in_list(address, &pos))
    {
        add_to_list(address, (uint8_t)TYPE_TEMP_SENSOR);
        pos = g_devs_list.number - 1;
    }

    temp_sensor = (temp_sensor_t *)&(g_memory.dev[pos]);

    temp_sensor->type = (uint8_t)TYPE_TEMP_SENSOR;
    temp_sensor->temp = (uint16_t)temp;
    temp_sensor->batt = batt;

    return NRF_SUCCESS;
}


/**
 * @brief Browses advertising packet for useful data (led_state).
                    If whole this data is found, function check if the device's address is already on the list.
                    If the device isn't on the list, it is added, and sensors count is incremented
 *
 * @param[in]  The device address.
 * @param[in]  Advertising packet data.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
uint8_t bt_handle_led_driver(ble_gap_addr_t * address, uint8_array_t * data)
{
    uint32_t       err_code;
    uint8_t        state;
    uint8_t        pos;
    led_driver_t * led_driver;

    // err_code = parse_temp(data, &temp);
    err_code = parse_service_data(data, 0x1866, (uint8_t *)&state, 1);

    if (err_code != NRF_SUCCESS)
        return NRF_ERROR_NOT_FOUND;

    if (!find_in_list(address, &pos))
    {
        add_to_list(address, TYPE_LED_DRIVER);
        pos = g_devs_list.number - 1;
    }

    led_driver = (led_driver_t *)&(g_memory.dev[pos]);

    led_driver->type  = (uint8_t)TYPE_LED_DRIVER;
    led_driver->state = (uint16_t)state;

    return NRF_SUCCESS;
}


/**
 * @brief Browses advertising packet for useful data (led_state).
                    If whole this data is found, function check if the device's address is already on the list.
                    If the device isn't on the list, it is added, and sensors count is incremented
 *
 * @param[in]  The device address.
 * @param[in]  Advertising packet data.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
uint32_t bt_connect(void)
{
    uint32_t            err_code;
    uint16_t            start_address;
    uint8_t             dev_idx;
    mb_rq_write_hdr_t * rq_packet;

    g_conn_dev_idx = -1; // means error

    rq_packet = (mb_rq_write_hdr_t *)g_buffer;

    start_address  = rq_packet->start[1];
    start_address |= (rq_packet->start[0] << 8);

    if (rq_packet->byte_count != (BT_DATA_LENGTH + 1))
        return NRF_ERROR_INVALID_LENGTH;

    dev_idx = start_address / (BT_DATA_LENGTH + 1);

    if ( dev_idx >= g_devs_list.number)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (g_devs_list.device[dev_idx].type & TYPE_WRITABLE_MASK)
    {
        err_code = sd_ble_gap_scan_stop();

        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
        g_scan_param.selective = 0;
        err_code               = sd_ble_gap_connect(&(g_devs_list.device[dev_idx].address),
                                                    &g_scan_param,
                                                    &g_connection_param);;

        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
        g_conn_dev_idx = dev_idx;
        return NRF_SUCCESS;
    }
    else
        return NRF_ERROR_NOT_SUPPORTED;
}


/**
 * @brief Update attribute value.
 *
 * @param[in] conn_handle Connection handle.
 *
 * @retval NRF_SUCCESS if operation was successful, error code otherwise.
 */
uint32_t bt_write_to_device(const uint16_t conn_handle)
{
    uint32_t            err_code;
    uint8_t             type;
    uint8_t           * data;
    uint8_t             length;
    mb_rq_write_hdr_t * rq_packet;

    if (g_conn_dev_idx < 0)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    rq_packet = (mb_rq_write_hdr_t *)g_buffer;

    type = g_devs_list.device[g_conn_dev_idx].type;

    switch ((device_type_e)type)
    {
        case TYPE_LED_DRIVER:
            data   = &(((led_driver_t *)(rq_packet->data))->state);
            length = sizeof (((led_driver_t *)(rq_packet->data))->state);
            break;

        default:
            return NRF_ERROR_NOT_SUPPORTED;
    }

    // prepare attribute's parameters structure
    ble_gattc_write_params_t params;
    params.handle   = HANDLE_LED_DRIVER_STATE;
    params.write_op = BLE_GATT_OP_WRITE_REQ;
    params.offset   = 0;
    params.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_CANCEL;
    params.len      = length;
    params.p_value  = data;

    err_code = sd_ble_gattc_write(conn_handle, &params);

    return err_code;
}


/**
 * @brief Disconnect from peer.
 *
 * @param[in] conn_handle Connection handle.
 *
 * @retval NRF_SUCCESS if disconnection was sucessfully initiated, error code otherwise.
 */
uint32_t bt_disconnect(const uint16_t conn_handle)
{
    if (conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        return sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    }

    return NRF_ERROR_NULL;
}


/**************************************************************
*   STATIC FUNCTIONS DEFINITIONS
**************************************************************/
/**
 * @brief Parse array to find useful data included in service data
 *
 * @param[in]  Advertisement packet or other data.
 * @param[out]  Found value.
 *
 * @retval Return NRF_SUCCES if service data was found, NRF_ERROR_NOT_FOUND otherwise.
 */
static uint8_t parse_service_data(uint8_array_t * adv_data,
                                  uint16_t        uuid,
                                  uint8_t       * value,
                                  uint8_t         length)
{
    uint8_t expr[3];

    expr[0] = 0x16; // service data
    expr[1] = *((uint8_t *)&uuid);
    expr[2] = *(((uint8_t *)&uuid) + 1);

    for (uint8_t i = 0; i < adv_data->size - 2 - length; ++i)
    {
        if ((adv_data->p_data[i] == expr[0]) && (adv_data->p_data[i + 1] == expr[1]) &&
            (adv_data->p_data[i + 2] == expr[2]))
        {
            for (uint8_t j = 0; j < length; ++j)
                *(value + j) = adv_data->p_data[i + j + 3];
            return NRF_SUCCESS;
        }
    }
    return NRF_ERROR_NOT_FOUND;
}


/**
 * @}
 */
