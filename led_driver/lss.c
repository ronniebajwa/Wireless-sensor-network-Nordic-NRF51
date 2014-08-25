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

#include "lss.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_l2cap.h"

#define OPCODE_LENGTH  1                                                   /**< Length of opcode inside LED State packet. */
#define HANDLE_LENGTH  2                                                   /**< Length of handle inside LED State packet. */
#define MAX_PACKET_LEN (BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum size of a transmitted packet. */

/*****************************************************************
 * STATIC FUNCTION DECLARATIONS
 ****************************************************************/

static uint32_t adv_params_char_add(ble_lss_t * p_lss, const ble_lss_init_t * p_lss_init);
static uint32_t led_state_char_add(ble_lss_t * p_lss, const ble_lss_init_t * p_lss_init);
static void     on_write(ble_lss_t * p_lss, ble_evt_t * p_ble_evt);
static void     save_adv_interval(ble_lss_t * p_lss, const uint8_t * data);
static void     on_connect(ble_lss_t * p_lss, const ble_evt_t * p_ble_evt);
static void     on_disconnect(ble_lss_t * p_lss, const ble_evt_t * p_ble_evt);


/****************************************************************
* NON-STATIC FUNCTIONS IMPLEMENTATIONS
****************************************************************/

/**@brief Function for initializing the LED State Service.
 *
 * @param[out]  p_lss       LED State Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_lss_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_lss_init(ble_lss_t * p_lss, const ble_lss_init_t * p_lss_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;


    // Initialize service structure
    p_lss->evt_handler             = p_lss_init->evt_handler;
    p_lss->conn_handle             = BLE_CONN_HANDLE_INVALID;
    p_lss->dev_params.adv_interval = DEFAULT_ADV_INTERVAL;
    p_lss->dev_params.state        = LED_INIT_STATE;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, MY_UUID_LED_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_lss->service_handle);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add heart rate measurement characteristic
    err_code = led_state_char_add(p_lss, p_lss_init);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add heart rate measurement characteristic
    err_code = adv_params_char_add(p_lss, p_lss_init);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the LED State Service.
 *
 * @param[in]   p_lss      LED State Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_lss_on_ble_evt(ble_lss_t * p_lss, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_lss, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_lss, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_lss, p_ble_evt);
            break;

        default:
            break;
    }
}


/****************************************************************
* STATIC FUNCTIONS IMPLEMENTATIONS
****************************************************************/

/**@brief Function for handling the Connect event.
 *
 * @param[in] p_lss       LED State Service structure.
 * @param[in] p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_lss_t * p_lss, const ble_evt_t * p_ble_evt)
{
    p_lss->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in] p_lss       LED State Service structure.
 * @param[in] p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_lss_t * p_lss, const ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_lss->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling write events to the LED State characteristic.
 *
 * @param[in] p_lss         LED State Service structure.
 * @param[in] p_evt_write   Write event received from the BLE stack.
 */
static void on_lss_cccd_write(ble_lss_t * p_lss, ble_gatts_evt_write_t * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_lss->evt_handler != NULL)
        {
            ble_lss_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_LSS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_LSS_EVT_NOTIFICATION_DISABLED;
            }

            p_lss->evt_handler(p_lss, &evt);
        }
    }
}


/**@brief Function to set advertising interval.
 *
 * @param[in] p_lss         LED State Service structure.
 * @param[in] p_evt_write   Write event received from the BLE stack.
 */
static void save_adv_interval(ble_lss_t * p_lss, const uint8_t * data)
{
    uint16_t length;
    uint16_t interval;

    interval  = data[0] & 0xFF;
    interval |= (data[1] << 8) & 0xFF00;

    // if interval value is not in proper range, change it's value and update in gatts
    if ((interval < BLE_GAP_ADV_INTERVAL_MIN) || (interval > BLE_GAP_ADV_INTERVAL_MAX))
    {
        if(interval < BLE_GAP_ADV_INTERVAL_MIN) interval = BLE_GAP_ADV_INTERVAL_MIN;
        else interval = BLE_GAP_ADV_INTERVAL_MAX;

        length   = sizeof (interval);
        sd_ble_gatts_value_set(p_lss->lss_ai_handle.value_handle,
                               0,
                               &length,
                               (uint8_t *)&(p_lss->dev_params.adv_interval));
    }

    p_lss->dev_params.adv_interval = interval;
}


/**@brief Function for handling the write event.
 *
 * @param[in] p_lss       LED State Service structure.
 * @param[in] p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_lss_t * p_lss, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    // save new LED state
    if (p_evt_write->handle == p_lss->lss_ls_handle.value_handle)
    {
        p_lss->dev_params.state = (led_color_e)(p_evt_write->data[0]);
    }

    // change advertising interval
    if (p_evt_write->handle == p_lss->lss_ai_handle.value_handle)
    {
        save_adv_interval(p_lss, p_evt_write->data);
    }

    if (p_evt_write->handle == p_lss->lss_ls_handle.cccd_handle)
    {
        on_lss_cccd_write(p_lss, p_evt_write);
    }
}


/**@brief Function for adding characteristic the LED State characteristic.
 *
 * @param[in]   p_lss        LED State Service structure.
 * @param[in]   p_lss_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t led_state_char_add(ble_lss_t * p_lss, const ble_lss_init_t * p_lss_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             char_init_val = (uint8_t)LED_INIT_STATE;
    uint8_t             char_length   = sizeof (char_init_val);

    memset(&cccd_md, 0, sizeof (cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.write_perm = p_lss_init->lss_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof (char_md));

    char_md.char_props.write_wo_resp = 1;
    char_md.char_props.write         = 1;
    char_md.char_props.read          = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = &cccd_md;
    char_md.p_sccd_md                = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, MY_UUID_LED_STATE_CHAR);

    memset(&attr_md, 0, sizeof (attr_md));

    attr_md.read_perm  = p_lss_init->lss_attr_md.read_perm;
    attr_md.write_perm = p_lss_init->lss_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof (attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = char_length;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = MAX_PACKET_LEN;
    attr_char_value.p_value   = &char_init_val;

    return sd_ble_gatts_characteristic_add(p_lss->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_lss->lss_ls_handle);
}


/**@brief Function for adding the Advertisement Parameters characteristic.
 *
 * @param[in]   p_lss        LED State Service structure.
 * @param[in]   p_lss_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t adv_params_char_add(ble_lss_t * p_lss, const ble_lss_init_t * p_lss_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             char_init_val = p_lss->dev_params.adv_interval;
    uint8_t             char_length   = sizeof (char_init_val);

    memset(&cccd_md, 0, sizeof (cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.write_perm = p_lss_init->lss_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof (char_md));

    char_md.char_props.write_wo_resp = 1;
    char_md.char_props.write         = 1;
    char_md.char_props.read          = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = &cccd_md;
    char_md.p_sccd_md                = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, MY_UUID_ADV_INTERVAL_CHAR);

    memset(&attr_md, 0, sizeof (attr_md));

    attr_md.read_perm  = p_lss_init->lss_attr_md.read_perm;
    attr_md.write_perm = p_lss_init->lss_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof (attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = char_length;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = MAX_PACKET_LEN;
    attr_char_value.p_value   = &char_init_val;

    return sd_ble_gatts_characteristic_add(p_lss->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_lss->lss_ai_handle);
}
