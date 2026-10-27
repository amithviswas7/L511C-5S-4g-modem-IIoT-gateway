
#include <stdint.h>
#include <stddef.h>

#include "modbus_rtu.h"
#include "mbtk_uart.h"
#include "mbtk_gpio.h"
#include "mbtk_comm_api.h"
#include "mbtk_api.h"
#include "modbus_rtu_timing.h"
/* =====================================================
   MODBUS TEST CONFIGURATION
   ===================================================== */

#define MODBUS_RESPONSE_TIMEOUT_POLLS  400U
#define MODBUS_POLL_INTERVAL_MS          1U

#define MODBUS_DEMO_SLAVE                1U
#define MODBUS_DEMO_START                100U
#define MODBUS_DEMO_QUANTITY             8U

#define MODBUS_REQUEST_SIZE              8U
#define MODBUS_EXPECTED_RESPONSE_SIZE    6U
#define MODBUS_MAX_RESPONSE_SIZE       256U

static uint8_t modbus_request[8];
static size_t modbus_request_len;

static uint8_t modbus_response[MODBUS_MAX_RESPONSE_SIZE];
static size_t modbus_response_len;

static uint8_t modbus_coils[MODBUS_DEMO_QUANTITY];
static uint8_t modbus_exception_code;

static uint8_t response_buffer[64];

mbtk_taskref mbtk_test_task_ref;

static mbtk_pin_num_enum tx_pin = mbtk_pin_18;
static mbtk_pin_num_enum rx_pin = mbtk_pin_17;

static OL_UART_DCB uart_dcb = {0};

/* =====================================================
   FUNCTION DECLARATIONS
   ===================================================== */

void init_cdc_uart(void);
void cdc_uart_printf(const char *fmt, ...);
void app_open_task(void *argv);

int gpio_config(void);
int uart_config(void);

/* =====================================================
   GPIO CONFIGURATION
   ===================================================== */

int gpio_config(void)
{
    mbtk_pin_config_struct config = {0};

    config.gpio_af_num = mbtk_gpio_config_maf1;
    config.gpio_pull = mbtk_gpio_config_pull_high;
    config.gpio_sleep = mbtk_gpio_config_sleep_none;
    config.gpio_edge = mbtk_gpio_config_edge_none;

    if (ol_pin_config(tx_pin, &config) != 0)
    {
        cdc_uart_printf("[ERROR] TX pin configuration failed\r\n");
        return -1;
    }

    if (ol_pin_config(rx_pin, &config) != 0)
    {
        cdc_uart_printf("[ERROR] RX pin configuration failed\r\n");
        return -1;
    }

    return 0;
}

/* =====================================================
   UART CONFIGURATION: 9600, 8-N-1
   ===================================================== */

int uart_config(void)
{
    int err;

    err = ol_Uart_GetDcb(OL_UART_PORT_STUART, &uart_dcb);

    if (err != OL_UART_RC_OK)
    {
        cdc_uart_printf(
            "[ERROR] ol_Uart_GetDcb: %d\r\n", err);
        return -1;
    }

    uart_dcb.baudRate = OL_UART_BAUD_9600;
    uart_dcb.numDataBits = OL_UART_WORD_LEN_8;
    uart_dcb.parityBitType = OL_UART_NO_PARITY_BITS;
    uart_dcb.stopBit = OL_UART_ONE_STOP_BIT;
    uart_dcb.flowControl = false;

    err = ol_Uart_SetDcb(OL_UART_PORT_STUART, &uart_dcb);

    if (err != OL_UART_RC_OK)
    {
        cdc_uart_printf(
            "[ERROR] ol_Uart_SetDcb: %d\r\n", err);
        return -1;
    }

    err = ol_Uart_Open(OL_UART_PORT_STUART);

    if (err != OL_UART_RC_OK)
    {
        cdc_uart_printf(
            "[ERROR] ol_Uart_Open: %d\r\n", err);
        return -1;
    }

    cdc_uart_printf("[UART] STUART ready: 9600 8-N-1\r\n");

    return 0;
}

/* =====================================================
   PARSE MODBUS FC01 RESPONSE
   ===================================================== */

static int modbus_process_response(void)
{
    int result;
    size_t i;

    result = modbus_parse_response(
        modbus_request,
        modbus_request_len,
        modbus_response,
        modbus_response_len,
        NULL,
        0U,
        modbus_coils,
        MODBUS_DEMO_QUANTITY,
        &modbus_exception_code
    );

    if (result == 0)
    {
        cdc_uart_printf("[MODBUS RX] Valid FC01 response: ");

        for (i = 0U; i < MODBUS_DEMO_QUANTITY; i++)
        {
            cdc_uart_printf(
                "%u ",
                (unsigned int)modbus_coils[i]);
        }

        cdc_uart_printf("\r\n");
        return 0;
    }

    if (result == 1)
    {
        cdc_uart_printf(
            "[MODBUS RX] Exception response: 0x%02X\r\n",
            (unsigned int)modbus_exception_code);

        return 1;
    }

    cdc_uart_printf(
        "[MODBUS RX] Invalid response, parser result=%d\r\n",
        result);

    return -1;
}

/* =====================================================
   FC01 REQUEST / RESPONSE TEST

   No timing calculator or RTU frame assembler is used.
   For quantity=8, both a normal FC01 response and an
   exception response are 5 bytes long.
   ===================================================== */

static int modbus_fc01_demo(void)
{
    size_t received_total = 0U;
    unsigned int poll_count;
    unsigned int i;
    u16 bytes_read;
    

    /* Build request: slave 1, start 0, quantity 8. */
    modbus_request_len = (size_t)modbus_build_read_coils(
        MODBUS_DEMO_SLAVE,
        MODBUS_DEMO_START,
        MODBUS_DEMO_QUANTITY,
        modbus_request,
        sizeof(modbus_request));

    if (modbus_request_len != MODBUS_REQUEST_SIZE)
    {
        cdc_uart_printf(
            "[MODBUS] Request build failed; length=%u\r\n",
            (unsigned int)modbus_request_len);
        return -1;
    }

    /*
     * Drain any bytes already waiting in the UART receive
     * buffer before starting a new transaction.
     */
    for (i = 0U; i < 16U; i++)
    {
        bytes_read = 0U;

        (void)ol_Uart_Read(
            OL_UART_PORT_STUART,
            response_buffer,
            sizeof(response_buffer),
            &bytes_read);

        if (bytes_read == 0U)
        {
            break;
        }
    }

    modbus_response_len = 0U;
    received_total = 0U;

    cdc_uart_printf("[MODBUS TX] ");

    for (i = 0U; i < (unsigned int)modbus_request_len; i++)
    {
        cdc_uart_printf("%02X ", modbus_request[i]);
    }

    cdc_uart_printf("\r\n");

    /* Send the complete Modbus request. */
    (void)ol_Uart_Write(
        OL_UART_PORT_STUART,
        modbus_request,
        (unsigned)modbus_request_len);

   

    /*
     * Receive exactly 5 bytes for this FC01 test.
     *
     * The poll-count timeout is an approximate application
     * timeout, not a precise timing measurement.
     */
    for (poll_count = 0U;
         poll_count < MODBUS_RESPONSE_TIMEOUT_POLLS;
         poll_count++)
    {
        bytes_read = 0U;

        ol_Uart_Read(
            OL_UART_PORT_STUART,
            response_buffer,
            sizeof(response_buffer),
            &bytes_read);

        for (i = 0U;
             i < (unsigned int)bytes_read;
             i++)
        {
            if (received_total < sizeof(modbus_response))
            {
                modbus_response[received_total++] =
                    response_buffer[i];
            }
            else
            {
                cdc_uart_printf(
                    "[MODBUS] Response buffer overflow\r\n");
                return -3;
            }
        }

        /*
         * For FC01 quantity 8:
         * normal response = 5 bytes
         * exception response = 5 bytes
         */
        if (received_total >= MODBUS_EXPECTED_RESPONSE_SIZE)
        {
            modbus_response_len = received_total;

            cdc_uart_printf("[MODBUS RX] ");

            for (i = 0U;
                 i < (unsigned int)modbus_response_len;
                 i++)
            {
                cdc_uart_printf(
                    "%02X ",
                    modbus_response[i]);
            }

            cdc_uart_printf("\r\n");

            return modbus_process_response();
        }

        ol_os_task_sleep(MODBUS_POLL_INTERVAL_MS);
    }

    cdc_uart_printf(
        "[MODBUS] Response timeout; received %u of %u bytes\r\n",
        (unsigned int)received_total,
        MODBUS_EXPECTED_RESPONSE_SIZE);

    return -4;
}



/* =====================================================
   ADDITIONAL MODBUS TEST BUFFERS
   ===================================================== */

static uint8_t modbus_discrete_inputs[8];
static uint16_t modbus_registers[8];

static uint8_t modbus_multiple_coils[2] = {
    0x55U,  /* Addresses 100–107: 1,0,1,0,1,0,1,0 */
    0x55U   /* Address 108 = 1, address 109 = 0 */
};

static uint8_t modbus_request_FC16[25] ;

static uint16_t modbus_multiple_registers[8] =
{
    100U, 101U, 102U, 103U,
    104U, 105U, 106U, 107U
};


/* =====================================================
   COMMON REQUEST TRANSMIT / RESPONSE RECEIVE HELPER

   Reuses the existing global buffers.
   Does not parse the response.
   ===================================================== */


static int modbus_run_transaction(
    const uint8_t *request,
    size_t request_len,
    size_t expected_normal_response_len)
{
    size_t received_total = 0U;
    size_t target_len;
    size_t i;
    unsigned int poll_count;
    u16 bytes_read;
    int write_result;

    if ((request == NULL) ||
        (request_len == 0U) ||
        (expected_normal_response_len > sizeof(modbus_response)))
    {
        cdc_uart_printf("[MODBUS] Invalid transaction length\r\n");
        return -1;
    }

    /* Drain bytes left over from a previous transaction. */
    for (i = 0U; i < 16U; i++)
    {
        bytes_read = 0U;

        (void)ol_Uart_Read(
            OL_UART_PORT_STUART,
            response_buffer,
            sizeof(response_buffer),
            &bytes_read);

        if (bytes_read == 0U)
        {
            break;
        }
    }

    modbus_response_len = 0U;

    cdc_uart_printf("[MODBUS TX] ");

    for (i = 0U; i < request_len; i++)
    {
        cdc_uart_printf("%02X ", request[i]);
    }

    cdc_uart_printf("\r\n");

    write_result = ol_Uart_Write(
        OL_UART_PORT_STUART,
        (uint8_t *)request,
        (unsigned)request_len);

  

    target_len = expected_normal_response_len;

    for (poll_count = 0U;
         poll_count < MODBUS_RESPONSE_TIMEOUT_POLLS;
         poll_count++)
    {
        bytes_read = 0U;

        (void)ol_Uart_Read(
            OL_UART_PORT_STUART,
            response_buffer,
            sizeof(response_buffer),
            &bytes_read);

        for (i = 0U; i < (size_t)bytes_read; i++)
        {
            if (received_total >= sizeof(modbus_response))
            {
                cdc_uart_printf("[MODBUS] Response buffer overflow\r\n");
                return -3;
            }

            modbus_response[received_total++] = response_buffer[i];

            if (received_total >= 2U &&
                (modbus_response[1] & 0x80U) != 0U)
            {
                target_len = 5U;
            }

            if (received_total >= target_len)
            {
                modbus_response_len = target_len;

                cdc_uart_printf("[MODBUS RX] ");

                for (i = 0U; i < modbus_response_len; i++)
                {
                    cdc_uart_printf("%02X ", modbus_response[i]);
                }

                cdc_uart_printf("\r\n");
                return 0;
            }
        }

        ol_os_task_sleep(MODBUS_POLL_INTERVAL_MS);
    }

    cdc_uart_printf(
        "[MODBUS] Response timeout; received %u bytes\r\n",
        (unsigned int)received_total);

    return -4;
}

/* =====================================================
   FC02: READ DISCRETE INPUTS
   Slave 1, start 100, quantity 8
   Expected normal response: 6 bytes
   ===================================================== */

static int modbus_fc02_demo(void)
{
    int result;
    size_t i;

    result = modbus_build_read_discrete_inputs(
        MODBUS_DEMO_SLAVE,
        MODBUS_DEMO_START,
        MODBUS_DEMO_QUANTITY,
        modbus_request,
        sizeof(modbus_request));

    if (result != 8)
    {
        cdc_uart_printf("[FC02] Request build failed: %d\r\n", result);
        return -1;
    }

    modbus_request_len = (size_t)result;

    result =modbus_run_transaction(modbus_request, modbus_request_len, 6U);

    if (result != 0)
    {
        return result;
    }
      result = modbus_parse_response(
        modbus_request, modbus_request_len,
        modbus_response, modbus_response_len,
        NULL, 0U,
        modbus_discrete_inputs, MODBUS_DEMO_QUANTITY,
        &modbus_exception_code);

    if (result == 0)
    {
        cdc_uart_printf("[FC02] Discrete inputs: ");

        for (i = 0U; i < MODBUS_DEMO_QUANTITY; i++)
        {
            cdc_uart_printf("%u ", (unsigned int)modbus_discrete_inputs[i]);
        }

        cdc_uart_printf("\r\n");
        return 0;
    }

    if (result == 1)
    {
        cdc_uart_printf("[FC02] Exception: 0x%02X\r\n",
                        (unsigned int)modbus_exception_code);
        return 1;
    }

    cdc_uart_printf("[FC02] Parser error: %d\r\n", result);
    return -5;
}


    

/* =====================================================
   FC03: READ HOLDING REGISTERS
   Slave 1, start 100, quantity 8
   Expected normal response: 21 bytes
   ===================================================== */

static int modbus_fc03_demo(void)
{
    int result;
    size_t i;

    result = modbus_build_read_holding_registers(
        MODBUS_DEMO_SLAVE,
        MODBUS_DEMO_START,
        MODBUS_DEMO_QUANTITY,
        modbus_request,
        sizeof(modbus_request));

    if (result != 8)
    {
        cdc_uart_printf("[FC03] Request build failed: %d\r\n", result);
        return -1;
    }

    modbus_request_len = (size_t)result;

    result = modbus_run_transaction(modbus_request, modbus_request_len, 21U);

    if (result != 0)
    {
        return result;
    }

    result = modbus_parse_response(
        modbus_request, modbus_request_len,
        modbus_response, modbus_response_len,
        modbus_registers, MODBUS_DEMO_QUANTITY,
        NULL, 0U,
        &modbus_exception_code);

    if (result == 0)
    {
        cdc_uart_printf("[FC03] Holding registers:\r\n");

        for (i = 0U; i < MODBUS_DEMO_QUANTITY; i++)
        {
            cdc_uart_printf(
                "  Address %u = %u\r\n",
                (unsigned int)(MODBUS_DEMO_START + i),
                (unsigned int)modbus_registers[i]);
        }

        return 0;
    }

    if (result == 1)
    {
        cdc_uart_printf("[FC03] Exception: 0x%02X\r\n",
                        (unsigned int)modbus_exception_code);
        return 1;
    }

    cdc_uart_printf("[FC03] Parser error: %d\r\n", result);
    return -5;
}


/* =====================================================
   FC04: READ INPUT REGISTERS
   Slave 1, start 100, quantity 8
   Expected normal response: 21 bytes
   ===================================================== */

static int modbus_fc04_demo(void)
{
    int result;
    size_t i;

    result = modbus_build_read_input_registers(
        MODBUS_DEMO_SLAVE,
        MODBUS_DEMO_START,
        MODBUS_DEMO_QUANTITY,
        modbus_request,
        sizeof(modbus_request));

    if (result != 8)
    {
        cdc_uart_printf("[FC04] Request build failed: %d\r\n", result);
        return -1;
    }

    modbus_request_len = (size_t)result;

    result = modbus_run_transaction(modbus_request, modbus_request_len, 21U);

    if (result != 0)
    {
        return result;
    }

    result = modbus_parse_response(
        modbus_request, modbus_request_len,
        modbus_response, modbus_response_len,
        modbus_registers, MODBUS_DEMO_QUANTITY,
        NULL, 0U,
        &modbus_exception_code);

    if (result == 0)
    {
        cdc_uart_printf("[FC04] Input registers:\r\n");

        for (i = 0U; i < MODBUS_DEMO_QUANTITY; i++)
        {
            cdc_uart_printf(
                "  Address %u = %u\r\n",
                (unsigned int)(MODBUS_DEMO_START + i),
                (unsigned int)modbus_registers[i]);
        }

        return 0;
    }

    if (result == 1)
    {
        cdc_uart_printf("[FC04] Exception: 0x%02X\r\n",
                        (unsigned int)modbus_exception_code);
        return 1;
    }

    cdc_uart_printf("[FC04] Parser error: %d\r\n", result);
    return -5;
}


/* =====================================================
   FC05: WRITE SINGLE COIL
   Slave 1, address 100, ON
   Expected normal response: 8 bytes
   ===================================================== */

static int modbus_fc05_demo(void)
{
    int result;

    result = modbus_build_write_single_coil(
        MODBUS_DEMO_SLAVE,
        MODBUS_DEMO_START,
        1U,
        modbus_request,
        sizeof(modbus_request));

    if (result != 8)
    {
        cdc_uart_printf("[FC05] Request build failed: %d\r\n", result);
        return -1;
    }

    modbus_request_len = (size_t)result;

    result = modbus_run_transaction(modbus_request, modbus_request_len, 8U);

    if (result != 0)
    {
        return result;
    }

    result = modbus_parse_response(
        modbus_request, modbus_request_len,
        modbus_response, modbus_response_len,
        NULL, 0U,
        NULL, 0U,
        &modbus_exception_code);

    if (result == 0)
    {
        cdc_uart_printf("[FC05] Coil address 100 set to ON; echo valid\r\n");
        return 0;
    }

    if (result == 1)
    {
        cdc_uart_printf("[FC05] Exception: 0x%02X\r\n",
                        (unsigned int)modbus_exception_code);
        return 1;
    }

    cdc_uart_printf("[FC05] Parser error: %d\r\n", result);
    return -5;
}


/* =====================================================
   FC06: WRITE SINGLE REGISTER
   Slave 1, address 100, value 1234
   Expected normal response: 8 bytes
   ===================================================== */

static int modbus_fc06_demo(void)
{
    int result;

    result = modbus_build_write_single_register(
        MODBUS_DEMO_SLAVE,
        MODBUS_DEMO_START,
        1234U,
        modbus_request,
        sizeof(modbus_request));

    if (result != 8)
    {
        cdc_uart_printf("[FC06] Request build failed: %d\r\n", result);
        return -1;
    }

    modbus_request_len = (size_t)result;

    result = modbus_run_transaction(modbus_request, modbus_request_len, 6U);

    if (result != 0)
    {
        return result;
    }

    result = modbus_parse_response(
        modbus_request, modbus_request_len,
        modbus_response, modbus_response_len,
        NULL, 0U,
        NULL, 0U,
        &modbus_exception_code);

    if (result == 0)
    {
        cdc_uart_printf(
            "[FC06] Register address 100 written with 1234; echo valid\r\n");
        return 0;
    }

    if (result == 1)
    {
        cdc_uart_printf("[FC06] Exception: 0x%02X\r\n",
                        (unsigned int)modbus_exception_code);
        return 1;
    }

    cdc_uart_printf("[FC06] Parser error: %d\r\n", result);
    return -5;
}


/* =====================================================
   FC15 / 0x0F: WRITE MULTIPLE COILS
   Start address 100, quantity 10
   Pattern: 1,0,1,0,1,0,1,0,1,0
   Expected normal response: 8 bytes
   ===================================================== */


static uint8_t modbus_request_fc15[11];

static int modbus_fc15_demo(void)
{
    int result;
    size_t request_len;

    result = modbus_build_write_multiple_coils(
        MODBUS_DEMO_SLAVE,
        MODBUS_DEMO_START,
        16U,
        modbus_multiple_coils,
        sizeof(modbus_multiple_coils),
        modbus_request_fc15,
        sizeof(modbus_request_fc15));

    if (result != 11)
    {
        cdc_uart_printf(
            "[FC15] Request build failed: %d\r\n", result);
        return -1;
    }

    request_len = (size_t)result;

    result = modbus_run_transaction(
        modbus_request_fc15,
        request_len,
        8U);

    if (result != 0)
    {
        return result;
    }

    result = modbus_parse_response(
        modbus_request_fc15,
        request_len,
        modbus_response,
        modbus_response_len,
        NULL,
        0U,
        NULL,
        0U,
        &modbus_exception_code);

    if (result == 0)
    {
        cdc_uart_printf(
            "[FC15] Multiple-coil write echo valid\r\n");
        return 0;
    }

    if (result == 1)
    {
        cdc_uart_printf(
            "[FC15] Exception: 0x%02X\r\n",
            (unsigned int)modbus_exception_code);
        return 1;
    }

    cdc_uart_printf(
        "[FC15] Parser error: %d\r\n", result);
    return -5;
}



/* =====================================================
   FC16 / 0x10: WRITE MULTIPLE REGISTERS
   Start address 100, quantity 8
   Values: 100 through 107
   Expected normal response: 8 bytes
   ===================================================== */

static int modbus_fc16_demo(void)
{
    int result;

    result = modbus_build_write_multiple_registers(
        MODBUS_DEMO_SLAVE,
        MODBUS_DEMO_START,
        MODBUS_DEMO_QUANTITY,
        modbus_multiple_registers,
        modbus_request_FC16,
        sizeof(modbus_request_FC16));

    if (result != 25)
    {
        cdc_uart_printf("[FC16] Request build failed: %d\r\n", result);
        return -1;
    }

    modbus_request_len = (size_t)result;

    result = modbus_run_transaction(modbus_request_FC16, modbus_request_len, 8U);

    if (result != 0)
    {
        return result;
    }

    result = modbus_parse_response(
        modbus_request_FC16, modbus_request_len,
        modbus_response, modbus_response_len,
        NULL, 0U,
        NULL, 0U,
        &modbus_exception_code);

    if (result == 0)
    {
        cdc_uart_printf("[FC16] Multiple-register write echo valid\r\n");
        return 0;
    }

    if (result == 1)
    {
        cdc_uart_printf("[FC16] Exception: 0x%02X\r\n",
                        (unsigned int)modbus_exception_code);
        return 1;
    }

    cdc_uart_printf("[FC16] Parser error: %d\r\n", result);
    return -5;
}


/* =====================================================
   APPLICATION INITIALIZATION
   ===================================================== */

void user_app_init(open_api_table *api_table)
{
    mbtk_api_init(api_table);
    init_cdc_uart();

    cdc_uart_printf("\r\n=== L511 MODBUS FC01 LIBRARY TEST ===\r\n");

    if (gpio_config() != 0)
    {
        cdc_uart_printf("[ERROR] GPIO configuration failed\r\n");
        return;
    }

    if (uart_config() != 0)
    {
        cdc_uart_printf("[ERROR] UART configuration failed\r\n");
        return;
    }

    ol_os_task_creat(
        &mbtk_test_task_ref,
        NULL,
        50 * 1024,
        220,
        "modbus_fc01_demo",
        app_open_task,
        NULL
    );
}

/* =====================================================
   APPLICATION TASK
   ===================================================== */

void app_open_task(void *argv)
{
    (void)argv;

    cdc_uart_printf("[MODBUS] Starting FC01 master test\r\n");

    while (1)
    {
       modbus_fc16_demo();

        /* Separate repeated requests during testing. */
        ol_os_task_sleep(5*200);
    }
}
