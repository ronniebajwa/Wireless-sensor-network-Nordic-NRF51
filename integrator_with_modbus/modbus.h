#ifndef __MODBUS_H
#define __MODBUS_H
/*

   MODBUS SLAVE LIBRARY

 */
#include <inttypes.h>
#include "devices.h"
#include "btble4.h"

#define MAX_BUFFER_SIZE         32    /**< Determines maximum UART buffer size. */

#define MB_DEVICE_ADDRESS       1     /**< Device's number on MODBUS. */
#define MB_RSP_READ_HDR_LENGTH  2     /**< MODBUS read multiple response header size */
#define MB_RSP_WRITE_HDR_LENGTH 5     /**< MODBUS write multiple response header size */
#define MB_RSP_ID_HDR_LENGTH    7     /**< MODBUS read device id response header size */
#define MB_RSP_ERROR_HDR_LENGTH 2     /**< MODBUS error response header size */
#define MB_ERROR_SHIFT          0x80  /**< shift number used to calculete error code: err_code = function + error_shift */
#define MB_MAX_REGS_QUANTITY    0x7B  /**< Value to indicate maximum registers number to write/read*/

#define TIMER_PRESCALER         5     /**< Timer prescaler for packet_timeout */
#define TIMEOUT_TICKS           120   /**< Timer's counter value to indicate packet_timeout */

#define VENDOR_NAME    "Nordic"       /**< MODBUS Vendor Name */
#define PRODUCT_CODE   "BLE4"         /**< MODBUS Product Code */
#define MINOR_REVISION "0.1"          /**< MODBUS Minor Revision  */
#define OBJECTS_NUMBER 3              /**< MNumber of objects to return as response to Read Device ID request  */

/*//////////////// MODBUS REGISTRY MAP /////////////////
   START
   ................................
   0x00     no of available devs     (R)
   ................................
   0x01     sensor 1 type            (R)
   0x02     sensor 1 data byte 1     (R/W)
   0x03     sensor 1 data byte 2     (R/W)
   0x04     sensor 1 data byte 3     (R/W)
   0x05     sensor 1 data byte 4     (R/W)
   0x06     sensor 1 data byte 5     (R/W)
   0x07     sensor 1 data byte 6     (R/W)
   0x08     sensor 1 data byte 7     (R/W)
   ................................
   0x09     sensor 2 type                           (R)
   0x0A     sensor 2 data byte 1     (R/W)
   0x0B     sensor 2 data byte 2     (R/W)
   0x0C     sensor 2 data byte 3     (R/W)
   0x0D     sensor 2 data byte 4     (R/W)
   0x0E     sensor 2 data byte 5     (R/W)
   0x0F     sensor 2 data byte 6     (R/W)
   0x10     sensor 2 data byte 7     (R/W)
   ................................
   .
   .
   .
   ................................
   END
 */                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       //////////////////////////////////////////////

/***********************************************************
* ENUMS
***********************************************************/

typedef enum
{
    MB_RQ_NONE            = 0x00,
    MB_RQ_READ_REGISTERS  = 0x03,
    MB_RQ_WRITE_REGISTERS = 0x10,
    MB_RQ_READ_ID         = 0x2B
} mb_function_e;

typedef enum
{
    MB_EXC_NONE          = 0x00,
    MB_EXC_NOT_SUPPORTED = 0x01,
    MB_EXC_BAD_ADDRESS   = 0x02,
    MB_EXC_BAD_LENGTH    = 0x03,
    MB_EXC_OTHER         = 0x04
} mb_exception_e;

/***********************************************************
* STRUCTS
***********************************************************/

typedef struct
{
    uint8_t type;
    uint8_t data[BT_DATA_LENGTH];
} mb_bt_device_t;

typedef struct
{
    uint16_t       dev_number;
    mb_bt_device_t dev[BT_MAX_DEVICES];
} mb_bt_memory_t;

typedef struct // 0x03 read multiple registers header
{uint8_t address;
 uint8_t function;
 uint8_t start[2];
 uint8_t regs_quantity[2];
 uint8_t reserved[2]; } mb_rq_read_hdr_t;

typedef struct // 0x10 write multiple registers header
{uint8_t address;
 uint8_t function;
 uint8_t start[2];
 uint8_t regs_quantity[2];
 uint8_t byte_count;
 uint8_t data[1]; // changable size - depends on bytes_count
} mb_rq_write_hdr_t;

typedef struct // 0x2B get id header
{uint8_t address;
 uint8_t function;
 uint8_t mei_type;
 uint8_t access_type;
 uint8_t object_id;
 uint8_t reserved[3]; } mb_rq_id_hdr_t;

typedef struct // 0x03 read multiple registers header
{
    uint8_t function;
    uint8_t byte_count;
    uint8_t reserved[2];
} mb_rsp_read_hdr_t;

typedef struct // 0x10 write multiple registers header
{
    uint8_t function;
    uint8_t start[2];
    uint8_t regs_quantity[2];
} mb_rsp_write_hdr_t;

typedef struct // 0x2B get id header rsp
{
    uint8_t function;
    uint8_t mei_type;       // 0x0E
    uint8_t access_type;    // 0x01 - basic info
    uint8_t conformity_lvl; // ?
    uint8_t more_follows;   // 0x00 if it's last packet, 0xff if not
    uint8_t next_object;    // ID of the next object - 0x00
    uint8_t number;         // number of objects
    uint8_t reserved[1];
} mb_rsp_id_hdr_t;


typedef struct // error rsp header
{
    uint8_t error_code;
    uint8_t exception; // number of objects
    uint8_t reserved[2];
} mb_rsp_error_hdr_t;

/***********************************************************
* FUNCTIONS DECLARATIONS
***********************************************************/

void    mb_init(void);
uint8_t mb_send_error_rsp(uint8_t code, uint8_t exception);
uint8_t mb_send_write_rsp(void);
void    mb_set_ready_for_rq(void);


#endif // __MODBUS_H
