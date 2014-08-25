#ifndef __BTLE4_H
#define __BTLE4_H

#include <inttypes.h>
#include "ble.h"


#define  BT_MAX_DEVICES 7                         /**< Determines maximum number of BLE devices */
#define  BT_DATA_LENGTH 7                         /**< Determines device data size in MODBUS memory registers */

/************** HANDLES *************/

#define   HANDLE_LED_DRIVER_STATE        0x0B     /**< Driver State attribute handle in led_driver device */
#define   HANDLE_LED_DRIVER_ADV_INTERVAL 0x0D     /**< Advertisement Interval attribute handle in led_driver device */

/************** STRUCTS *************/

typedef struct
{
    uint8_t  type;
    uint16_t temp;
    uint16_t batt;
    uint8_t  reserved[3];
} temp_sensor_t;

typedef struct
{
    uint8_t  type;
    uint8_t  state;
    uint16_t adv_interval;
    uint8_t  reserved[4];
} led_driver_t;

typedef enum
{
    TYPE_TEMP_SENSOR   = 0x01,
    TYPE_WRITABLE_MASK = 0x80,
    TYPE_LED_DRIVER    = 0x82, // MSB bit denotes if a device is writable
} device_type_e;

typedef struct
{
    ble_gap_addr_t address;
    uint8_t        type;
    uint8_t        data[BT_DATA_LENGTH];
} bt_device_t;

typedef struct
{
    uint8_t     number;
    bt_device_t device[BT_MAX_DEVICES];
} bt_devices_list_t;

#endif // __BTLE4_H
