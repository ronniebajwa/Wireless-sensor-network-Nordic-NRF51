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

#include <stdint.h>
#include "boards.h"
#include "app_gpiote.h"
#include "pstorage.h"
#include "device_manager.h"
#include "app_timer.h"
#include "main.h"
#include "app_button.h"
#include "modbus.h"

bool m_memory_access_in_progress = false; /**< Flag to keep track of ongoing operations on persistent memory. */

/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    NVIC_SystemReset();
}


/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for initializing the GPIOTE module.
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}


/**@brief Function for handling button events.
 *
 * @param[in]   pin_no   The pin number of the button pressed.
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    if (button_action == APP_BUTTON_PUSH)
    {
        switch (pin_no)
        {
            case BUTTON_CLEAN_LIST:
                bt_clean_list(); // Clean sensors list
                break;

            default:
                APP_ERROR_HANDLER(pin_no);
                break;
        }
    }
}


/**@brief Function for initializing the button module.
 */
static void buttons_init(void)
{
    uint32_t err_code;
    // Configure HR_INC_BUTTON_PIN_NO and HR_DEC_BUTTON_PIN_NO as wake up buttons and also configure
    // for 'pull up' because the eval board does not have external pull up resistors connected to
    // the buttons.
    static app_button_cfg_t buttons[] =
    {
        {BUTTON_CLEAN_LIST, false, BUTTON_PULL, button_event_handler} // Note: This pin is also BONDMNGR_DELETE_BUTTON_PIN_NO
    };

    APP_BUTTON_INIT(buttons, sizeof (buttons) / sizeof (buttons[0]), BUTTON_DETECTION_DELAY, false);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details This function is responsible for managing possible quick connect->write->disconnect sequence.
 *          It to write characteristic and switch the device in scan mode and getting advertising packets.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t              err_code;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_WRITE_RSP:
            mb_send_write_rsp();
            err_code = bt_disconnect(p_ble_evt->evt.gap_evt.conn_handle);

            if (err_code != NRF_SUCCESS)
            {
                bt_scan_start();
                mb_set_ready_for_rq();
            }
            break;

        case BLE_GAP_EVT_CONNECTED:
            nrf_gpio_pin_set(LED_CONNECTED);
            err_code = bt_write_to_device(p_ble_evt->evt.gap_evt.conn_handle);

            if (err_code != NRF_SUCCESS)
            {
                err_code = bt_disconnect(p_ble_evt->evt.gap_evt.conn_handle);

                if (err_code != NRF_SUCCESS)
                {
                    bt_scan_start();
                    mb_set_ready_for_rq();
                }
            }
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_clear(LED_CONNECTED);
            bt_scan_start();
            break;

        case BLE_GAP_EVT_ADV_REPORT:
        {
            uint8_array_t  adv_data;
            ble_gap_addr_t address;

            // Initialize advertisement report for parsing.
            adv_data.p_data = (uint8_t *)p_gap_evt->params.adv_report.data;
            adv_data.size   = p_gap_evt->params.adv_report.dlen;
            address         = p_gap_evt->params.adv_report.peer_addr;

            bt_handle_temp_sensor(&address, &adv_data);
            bt_handle_led_driver(&address, &adv_data);
            break;
        }

        case BLE_GAP_EVT_TIMEOUT:
            nrf_gpio_pin_clear(LED_CONNECTED);
            bt_scan_start();
            mb_set_ready_for_rq();
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.
 */
static void on_sys_evt(uint32_t sys_evt)
{
    switch (sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        case NRF_EVT_FLASH_OPERATION_ERROR:

            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                bt_scan_start();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *  been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for System events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for initializing the Device Manager.
 *
 * @details Device manager is initialized here.
 */
static void device_manager_init(void)
{
    dm_init_param_t init_param;
    uint32_t        err_code;

    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void leds_init(void)
{
    nrf_gpio_cfg_output(LED_RQ_PENDING);
    nrf_gpio_cfg_output(LED_CONNECTED);
}


/** @brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}


int main(void)
{
    // Initialization of various modules.
    timers_init();
    leds_init();
    gpiote_init();
    buttons_init();

    ble_stack_init();
    scheduler_init();
    device_manager_init();

    mb_init();

    // Start scanning for peripherals
    bt_scan_start();

    for (;; )
    {
        app_sched_execute();
        power_manage();
    }
}


