/* Copyright (C) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
 
#include "modbus.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "app_timer.h"
#include "btble4.h"
#include <string.h>
#include "main.h"

/**************************************************************
*   GLOBAL VARIABLES
**************************************************************/

uint8_t        g_buffer[MAX_BUFFER_SIZE];    /**< RX/TX UART buffer */
uint8_t        g_bytes_in_rsp_buffer = 0;    /**< To indicate number of bytes to send in buffer */
                                           
uint8_t        g_rq_idx          = 0;        /**< Ancillary variable to cope with buffer management whilst receiving a request bytes in ISR. */
uint8_t        g_rsp_idx         = 0;        /**< Ancillary variable to cope with buffer management whilst sending a response bytes in ISR. */
uint8_t        g_rq_pending_flag = 0;        /**< This flag indicates that MODBUS request is being processed. */
                                           
app_timer_id_t m_packet_timeout_timer_id;    /**< Handle for timer which timeout packet receiving and start it's processing. */
mb_bt_memory_t g_memory;                     /**< MODBUS registers with sensors' data */

/**************************************************************
*   STATIC FUNCTIONS DECLARATIONS
**************************************************************/

static void      byte_received(void);
static uint8_t   check_CRC(const uint8_t * packet, int hdr_len);
static uint8_t   check_write_rq(void);
static uint16_t  get_CRC(const uint8_t * ptr, int len);
static void      packet_timeout_handler(void);
static void      packet_timeout_timer_init(void);
static void      packet_timeout_timer_reset(void);
static void      packet_timeout_timer_start(void);
static void      packet_timeout_timer_stop(void);
static uint8_t   prepare_id_rsp(void);
static uint8_t   prepare_read_rsp(void);
static void      request_handler(void * p_event_data, uint16_t event_size);
static void      send_byte(uint8_t byte);
static void      send_rsp_byte(void);
static void      start_rq_processing(void);

/***********************************************************
* NON-STATIC FUNCTION
***********************************************************/

/**
 * @brief Reset internal variables and set device ready to receive next request.
 *
 * @retval  MB_EXC_NONE is always returned. 
 */
void mb_set_ready_for_rq(void)
{
    g_bytes_in_rsp_buffer = 0;
    g_rq_pending_flag     = 0;
    g_rq_idx              = 0;
    g_rsp_idx             = 0;
    nrf_gpio_pin_clear(LED_RQ_PENDING);
}


/**
 * @brief TIMER2 RQ Handler - when timer timeouts, request's receiving is done and it's processing starts.
 *
 */
void TIMER2_IRQHandler(void)
{
    NRF_TIMER2->EVENTS_COMPARE[0] = 0; // clear flag

    packet_timeout_handler(); // process packet
}


/**
 * @brief UART0 Handler - managing with receiving and sending single bytes comprises packets
 *
 */
void UART0_IRQHandler(void)
{
    if (NRF_UART0->EVENTS_RXDRDY)
    {
        NRF_UART0->EVENTS_RXDRDY = 0;

        // check if previous request has been processed
        if ( g_rq_pending_flag )
        {
            NRF_UART0->RXD; // just to clear interrupt flag: clear RXDRDY & red RXD
        }
        else
        {
            byte_received();
        }
    }
    else if (NRF_UART0->EVENTS_TXDRDY)
    {
        NRF_UART0->EVENTS_TXDRDY = 0;
        nrf_gpio_cfg_input(TX_PIN_NUMBER, NRF_GPIO_PIN_PULLUP); // if byte sent, switch to HZ state, change to NO_PULL if several devices

        if ( g_rq_pending_flag )
        {
            send_rsp_byte();
        }
    }
}

/**
 * @brief UART initialization for MODBUS
 *
 * @retval NRF_SUCCESS if device was found, error code otherwise.
 */
void mb_init(void)
{
    uint32_t err_code;

/** @snippet [Configure UART RX and TX pin] */
    nrf_gpio_cfg_output(TX_PIN_NUMBER);
    nrf_gpio_cfg_input(RX_PIN_NUMBER, NRF_GPIO_PIN_NOPULL);

    NRF_UART0->PSELTXD = TX_PIN_NUMBER;
    NRF_UART0->PSELRXD = RX_PIN_NUMBER;
/** @snippet [Configure UART RX and TX pin] */
    NRF_UART0->INTENSET = (UART_INTENSET_RXDRDY_Set << UART_INTENSET_RXDRDY_Pos) |
                          (UART_INTENSET_TXDRDY_Set << UART_INTENSET_TXDRDY_Pos);

    NRF_UART0->BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud115200 << UART_BAUDRATE_BAUDRATE_Pos);
    NRF_UART0->ENABLE   = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);

    // Enable ADC interrupt
    err_code = sd_nvic_ClearPendingIRQ(UART0_IRQn);
    APP_ERROR_CHECK(err_code);

    err_code = sd_nvic_SetPriority(UART0_IRQn, NRF_APP_PRIORITY_LOW);
    APP_ERROR_CHECK(err_code);

    err_code = sd_nvic_EnableIRQ(UART0_IRQn);
    APP_ERROR_CHECK(err_code);

    NRF_UART0->TASKS_STARTTX = 1;
    NRF_UART0->TASKS_STARTRX = 1;
    NRF_UART0->EVENTS_RXDRDY = 0;
    NRF_UART0->EVENTS_TXDRDY = 0;

    packet_timeout_timer_init();
}


/**
 * @brief Initiates sending packet as a response to write request.
 *
 * @retval  MB_EXC_NONE is always returned. 
 */
uint8_t mb_send_write_rsp()
{
    mb_rq_write_hdr_t * rq_hdr;
    mb_rsp_write_hdr_t  rsp_hdr;
    uint16_t            crc;

    rq_hdr = (mb_rq_write_hdr_t *)&g_buffer;

    rsp_hdr.function         = rq_hdr->function;
    rsp_hdr.start[0]         = rq_hdr->start[0];
    rsp_hdr.start[1]         = rq_hdr->start[1];
    rsp_hdr.regs_quantity[0] = rq_hdr->regs_quantity[0];
    rsp_hdr.regs_quantity[1] = rq_hdr->regs_quantity[1];

    // copy packet header to buffer
    memcpy((uint8_t *)&g_buffer, &rsp_hdr, MB_RSP_WRITE_HDR_LENGTH);
    g_bytes_in_rsp_buffer = MB_RSP_WRITE_HDR_LENGTH;

    // calculate and copy crc to buffer
    crc = get_CRC((const uint8_t *)g_buffer, g_bytes_in_rsp_buffer);
    memcpy((uint8_t *)&g_buffer + g_bytes_in_rsp_buffer, &crc, sizeof (crc));
    g_bytes_in_rsp_buffer += sizeof (crc);

    send_rsp_byte();

    return MB_EXC_NONE;
}


/**
 * @brief Initiates sending response with error packet containing error code and exception.
 *
 * @param[in]  code      Error code.
 * @param[in]  exception Exception number.
 *
 * @retval  MB_EXC_NONE is always returned. 
 */
uint8_t mb_send_error_rsp(uint8_t code, uint8_t exception)
{
    mb_rsp_error_hdr_t rsp_hdr;
    uint16_t           crc;

    rsp_hdr.error_code = code;
    rsp_hdr.exception  = exception;

    // copy packet header to buffer
    memcpy((uint8_t *)&g_buffer, &rsp_hdr, MB_RSP_ERROR_HDR_LENGTH);
    g_bytes_in_rsp_buffer = MB_RSP_ERROR_HDR_LENGTH;

    // calculate and copy crc to buffer
    crc = get_CRC((const uint8_t *)g_buffer, g_bytes_in_rsp_buffer);
    memcpy((uint8_t *)&g_buffer + g_bytes_in_rsp_buffer, &crc, sizeof (crc));
    g_bytes_in_rsp_buffer += sizeof (crc);

    send_rsp_byte();

    return MB_EXC_NONE;
}


/**************************************************************
*   STATIC FUNCTIONS DEFINITIONS
**************************************************************/

/**
 * @brief    Function to process received byte.
 * @details  When first byte is received, function starts timer and resets it's counter when next bytes are received.
 *
 */
static void byte_received()
{
    g_buffer[g_rq_idx] = NRF_UART0->RXD; // read RXD and save in packet structure
    ++g_rq_idx;                          // increment pointer indicating where to save received data

    if (g_rq_idx == 1)
    {
        packet_timeout_timer_start();
    }
    else
    {
        packet_timeout_timer_reset(); // if start of packet - run timeout timer
    }
}


/**
 * @brief    Function to check if packet's CRC is correct.
 *
 * @retval   NRF_SUCCESS if crc matches, NRF_ERROR_INVALID_DATA otherwise.
 */
static uint8_t check_CRC(const uint8_t * packet, int hdr_len)
{
    uint16_t crc = (uint16_t)(packet[hdr_len - 1] << 8) | (uint16_t)packet[hdr_len - 2];

    if (get_CRC(packet, hdr_len - 2) == crc)
        return NRF_SUCCESS;
    else
        return NRF_ERROR_INVALID_DATA;
}


/**
 * @brief    Function to check if packet's CRC is correct.
 *
 * @retval   MB_EXC_NONE if write request is correct, modbus exception code otherwise.
 */
static uint8_t check_write_rq()
{
    mb_rq_write_hdr_t * rq_hdr;
    uint8_t           * data_to_send;

    rq_hdr = (mb_rq_write_hdr_t *)&g_buffer;

    // check if registers quantity is valid
    if ((rq_hdr->regs_quantity[1] == 0) ||
        rq_hdr->regs_quantity[1] > MB_MAX_REGS_QUANTITY ||
        rq_hdr->regs_quantity[0] > 0)
        return MB_EXC_BAD_LENGTH;

    // 1 register -> 2 bytes
    if (rq_hdr->byte_count != 2 * rq_hdr->regs_quantity[1])
    {
        return NRF_ERROR_INVALID_DATA;
    }

    data_to_send = (uint8_t *)&g_memory +
                   ((uint32_t)rq_hdr->start[1] | (uint32_t)(rq_hdr->start[0] << 8));

    // check if address lav range doesn't exceed available area
    if (((data_to_send + rq_hdr->byte_count) > ((uint8_t *)&g_memory + sizeof (mb_bt_memory_t))))
    {
        return MB_EXC_BAD_ADDRESS;
    }

    // check CRC
    if (check_CRC((const uint8_t *)&g_buffer, g_rq_idx) != NRF_SUCCESS)
    {
        return MB_EXC_OTHER;
    }

    // check if buffer will be exceeded
    if (MB_RSP_WRITE_HDR_LENGTH + 2 >= MAX_BUFFER_SIZE)
    {
        return MB_EXC_BAD_ADDRESS;
    }

    return MB_EXC_NONE;
}


/**
 * @brief    Function to return CRC code of packet.
 *
 * @param[in] ptr Pointer to data.
 * @param[in] len Length of data under pointer.
 *
 * @retval   MB_EXC_NONE if write request is correct, modbus exception code otherwise.
 */
static uint16_t get_CRC(const uint8_t * ptr, int len)
{
    uint16_t crc = 0xFFFF;
    uint8_t  pos, i;

    if (ptr != NULL) // if header and payload are not in the same structure

    {
        for (pos = 0; pos < len; pos++)
        {
            crc ^= (uint16_t)ptr[pos]; // XOR byte into least sig. byte of crc

            for (i = 8; i != 0; i--) // Loop over each bit
            {
                if ((crc & 0x0001) != 0) // If the LSB is set
                {
                    crc >>= 1; // Shift right and XOR 0xA001
                    crc  ^= 0xA001;
                }
                else            // Else LSB is not set
                    crc >>= 1;  // Just shift right
            }
        }
    }
    return crc;
}


/**
 * @brief    Function to handle packet timeout event.
 *
 */
static void packet_timeout_handler(void)
{
    start_rq_processing();
    packet_timeout_timer_stop();

    if (g_buffer[0] != MB_DEVICE_ADDRESS)
    {
        mb_set_ready_for_rq();
    }
    else
    {
        app_sched_event_put(NULL, NULL, &request_handler);
    }
}


/**
 * @brief Initialization of timer.
 *
 */
static void packet_timeout_timer_init()
{
    NRF_TIMER2->MODE      = TIMER_MODE_MODE_Timer;
    NRF_TIMER2->BITMODE   = TIMER_BITMODE_BITMODE_32Bit;
    NRF_TIMER2->PRESCALER = TIMER_PRESCALER;
    NRF_TIMER2->CC[0]     = TIMEOUT_TICKS;
    NRF_TIMER2->INTENSET  = TIMER_INTENSET_COMPARE0_Msk;
    NVIC_SetPriority(TIMER2_IRQn, 3);
    NVIC_EnableIRQ(TIMER2_IRQn);
}

/**
 * @brief Resetting timer's counter.
 *
 */
static void packet_timeout_timer_reset(void)
{
    NRF_TIMER2->TASKS_CLEAR = 1;
}


/**
 * @brief Starting timer.
 *
 */
static void packet_timeout_timer_start(void)
{
    NRF_TIMER2->TASKS_CLEAR = 1;
    NRF_TIMER2->TASKS_START = 1;
}


/**
 * @brief Stopping timer and clearing it's counter.
 *
 */
static void packet_timeout_timer_stop(void)
{
    NRF_TIMER2->TASKS_CLEAR = 1;
    NRF_TIMER2->TASKS_STOP  = 1;
}


/**
 * @brief Function to prepare MODBUS response for get device information request.
 *
 * @retval   MB_EXC_NONE if write request is correct, modbus exception code otherwise.
 */
static uint8_t prepare_id_rsp()
{
    mb_rq_id_hdr_t * rq_hdr;
    mb_rsp_id_hdr_t  rsp_hdr;
    uint16_t         crc;
    uint8_t          hdr_len = 7;

    rq_hdr = (mb_rq_id_hdr_t *)&g_buffer;

    // check CRC
    if (check_CRC((const uint8_t *)&g_buffer, g_rq_idx) != NRF_SUCCESS)
        return MB_EXC_OTHER;

    // copy header to the buffer
    rsp_hdr.function       = rq_hdr->function;
    rsp_hdr.mei_type       = rq_hdr->mei_type;
    rsp_hdr.access_type    = rq_hdr->access_type;
    rsp_hdr.conformity_lvl = rq_hdr->access_type;
    rsp_hdr.more_follows   = 0x00;
    rsp_hdr.next_object    = rq_hdr->object_id;
    rsp_hdr.number         = OBJECTS_NUMBER;

    memcpy((uint8_t *)&g_buffer, &rsp_hdr, MB_RSP_ID_HDR_LENGTH); // do not copy string delimiter '/0'

    // copy VENDOR_NAME to buffer
    g_bytes_in_rsp_buffer             = hdr_len;
    g_buffer[g_bytes_in_rsp_buffer++] = 0x00;
    g_buffer[g_bytes_in_rsp_buffer++] = sizeof (VENDOR_NAME);
    memcpy((uint8_t *)&g_buffer + g_bytes_in_rsp_buffer, VENDOR_NAME, sizeof (VENDOR_NAME) - 1); // do not copy string delimiter '/0'
    g_bytes_in_rsp_buffer += sizeof (VENDOR_NAME) - 1;

    // copy PRODUCT_CODE to buffer
    g_buffer[g_bytes_in_rsp_buffer++] = 0x01;
    g_buffer[g_bytes_in_rsp_buffer++] = sizeof (PRODUCT_CODE);
    memcpy((uint8_t *)&g_buffer + g_bytes_in_rsp_buffer, PRODUCT_CODE, sizeof (PRODUCT_CODE) - 1); // do not copy string delimiter '/0'
    g_bytes_in_rsp_buffer += sizeof (PRODUCT_CODE) - 1;

    // copy MINOR_REVISION to buffer
    g_buffer[g_bytes_in_rsp_buffer++] = 0x02;
    g_buffer[g_bytes_in_rsp_buffer++] = sizeof (MINOR_REVISION);
    memcpy((uint8_t *)&g_buffer + g_bytes_in_rsp_buffer, MINOR_REVISION, sizeof (MINOR_REVISION) -
           1); // do not copy string delimiter '/0'
    g_bytes_in_rsp_buffer += sizeof (MINOR_REVISION) - 1;

    crc = get_CRC((const uint8_t *)g_buffer, g_bytes_in_rsp_buffer);

    // copy crc to buffer
    memcpy((uint8_t *)&g_buffer + g_bytes_in_rsp_buffer, &crc, sizeof (crc));
    g_bytes_in_rsp_buffer += sizeof (crc);

    return MB_EXC_NONE;
}


/**
 * @brief Function to prepare MODBUS response for read request.
 *
 * @retval   MB_EXC_NONE if write request is correct, modbus exception code otherwise.
 */
static uint8_t prepare_read_rsp()
{
    mb_rq_read_hdr_t * rq_hdr;
    mb_rsp_read_hdr_t  rsp_hdr;
    uint8_t          * data_to_send;
    uint16_t           crc;

    rq_hdr = (mb_rq_read_hdr_t *)&g_buffer;

    // check if registers quantity is valid
    if ((rq_hdr->regs_quantity[1] == 0) ||
        rq_hdr->regs_quantity[1] > 0x7D ||
        rq_hdr->regs_quantity[0] > 0)
        return MB_EXC_BAD_LENGTH;

    // 1 register equals 2 bytes
    rsp_hdr.byte_count = 2 * rq_hdr->regs_quantity[1]; // regs_quantity[0] should be always == 0, regs_quantity[1] maximum == 0x7D

    data_to_send = (uint8_t *)&g_memory +
                   ((uint32_t)rq_hdr->start[1] | (uint32_t)(rq_hdr->start[0] << 8));

    // check if address lav range doesn't exceed available area
    if (((data_to_send + rsp_hdr.byte_count) > ((uint8_t *)&g_memory + sizeof (mb_bt_memory_t))))
        return MB_EXC_BAD_ADDRESS;

    rsp_hdr.function = rq_hdr->function;

    // check CRC
    if (check_CRC((const uint8_t *)&g_buffer, g_rq_idx) != NRF_SUCCESS)
        return NRF_ERROR_INVALID_DATA;  // g_rq_idx hold number of received bytes

    // check if buffer will be exceeded
    if (MB_RSP_READ_HDR_LENGTH + rsp_hdr.byte_count + 2 >= MAX_BUFFER_SIZE)
        return MB_EXC_BAD_ADDRESS;

    // copy packet header to buffer
    memcpy((uint8_t *)&g_buffer, &rsp_hdr, MB_RSP_READ_HDR_LENGTH);
    g_bytes_in_rsp_buffer = MB_RSP_READ_HDR_LENGTH;

    // copy registers' data to buffer
    memcpy((uint8_t *)&g_buffer + g_bytes_in_rsp_buffer, data_to_send, rsp_hdr.byte_count);
    g_bytes_in_rsp_buffer += rsp_hdr.byte_count;

    // calculate and copy crc to buffer
    crc = get_CRC((const uint8_t *)g_buffer, g_bytes_in_rsp_buffer);
    memcpy((uint8_t *)&g_buffer + MB_RSP_READ_HDR_LENGTH + rsp_hdr.byte_count, &crc, sizeof (crc));
    g_bytes_in_rsp_buffer += sizeof (crc);

    return MB_EXC_NONE;
}


/**
 * @brief   Function to process MODBUS request.
 * @details Retrieves fun code and despatches packet processing to proper function.
 *          The function is added to app_scheduler queue when request received is finished.
 *
 * @param[in] p_event_data Not used. Demanded for app_scheduler compatibility.
 * @param[in] event size   Not used. Demanded for app_scheduler compatibility.
 */
static void request_handler(void * p_event_data, uint16_t event_size)
{
    uint8_t  fun;
    uint32_t bt_error;
    uint8_t  exception;

    fun = g_buffer[1];  // second byte in request is always fun code.

    switch (fun)
    {
        case MB_RQ_READ_REGISTERS:
            exception = prepare_read_rsp();
            break;

        case MB_RQ_WRITE_REGISTERS:
            exception = check_write_rq();

            if (exception == MB_EXC_NONE)
            {
                bt_error = bt_connect();

                if (bt_error == NRF_SUCCESS)
                    return;
								
                exception = MB_EXC_OTHER;
            }
            break;

        case MB_RQ_READ_ID:
            exception = prepare_id_rsp();
            break;

        default: exception = MB_EXC_NOT_SUPPORTED; 
            break;
    }

    if (exception)
    {
        mb_send_error_rsp(fun + MB_ERROR_SHIFT, exception);
    }
    else
    {
        send_rsp_byte();
    }
}


/**
 * @brief   Low-level function to send single byte through UART.
 * @details Due to multi-slave architecture TX pin of slave MODBUS device is push-pull
 *          only when this device is sure to be the one which is transmitting. After having send
 *          response packet, TX is configure as input Hi-Z pin.
 *
 * @param[in] send_byte byte to be send through UART.
 */
static void send_byte(uint8_t byte)
{
    nrf_gpio_cfg_output(TX_PIN_NUMBER);
    NRF_UART0->TXD = byte;
}


/**
 * @brief   Function to send next byte from buffer through UART.
 */
static void send_rsp_byte(void)
{
    if (g_rsp_idx < g_bytes_in_rsp_buffer)
    {
        send_byte(g_buffer[g_rsp_idx]);
        ++g_rsp_idx;
    }
    else
    {
        mb_set_ready_for_rq();
    }
}


/**
 * @brief Function which should be called first to start request processing.
 */
static void start_rq_processing(void)
{
    g_rq_pending_flag = 1;
    nrf_gpio_pin_set(LED_RQ_PENDING);
}


