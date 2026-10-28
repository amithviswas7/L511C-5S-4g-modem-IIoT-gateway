#include "mbtk_gpio.h"
#include "mbtk_api.h"
#include "mbtk_comm_api.h"
#include "menu_demo_api.h"
#include "mbtk_uart.h"
//in d drive folder L511_PC_Agent consists of python script to send data (wifi strenght percentage, cpu utilization)
#include "ol_nw_api.h"
#include "ol_nw_pub.h"
#include "mbtk_datacall_api.h"
#include "ol_mqttclient.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>


#include "modbus_rtu.h"

/* Modbus RTU frame storage */
#define MODBUS_FRAME_MAX       256U

/* UART receive scratch buffer */


/* Poll interval: confirm the sleep unit in your SDK */
#define MODBUS_POLL_INTERVAL_MS  1U

/* Example timeout: 400 polls at 1 ms = about 400 ms,
   excluding execution and scheduling overhead. */
#define MODBUS_RESPONSE_TIMEOUT_POLLS  400U

/* =====================================================
   MODBUS REQUEST QUEUE
   ===================================================== */

#define MODBUS_QUEUE_DEPTH       8U
#define MQTT_PACKET_ID_SIZE      4U
#define MQTT_MAX_PAYLOAD         (MQTT_PACKET_ID_SIZE + MODBUS_FRAME_MAX)

typedef struct
{
    uint32_t packet_id;
    size_t frame_len;
    uint8_t frame[MODBUS_FRAME_MAX];
} modbus_queued_request_t;

static modbus_queued_request_t
    modbus_request_queue[MODBUS_QUEUE_DEPTH];

/*
 * Single producer: MQTT callback.
 * Single consumer: app_open_task().
 *
 * Unsigned monotonic counters allow all eight slots to be used.
 */
static volatile uint32_t modbus_queue_head = 0U;
static volatile uint32_t modbus_queue_tail = 0U;

/* Compiler memory-ordering barrier for ARM Compiler 5 / GCC. */
#if defined(__CC_ARM)
#include <intrinsics.h>
#define MODBUS_QUEUE_BARRIER() __schedule_barrier()
#elif defined(__GNUC__)
#define MODBUS_QUEUE_BARRIER() __asm__ volatile ("" ::: "memory")
#else
#error "Define MODBUS_QUEUE_BARRIER for this compiler."
#endif

/* Queue function declarations */
static int modbus_queue_push_payload(
    const uint8_t *payload,
    size_t payload_len);

static int modbus_queue_pop(
    modbus_queued_request_t *request);

static void modbus_process_queued_request(
    const modbus_queued_request_t *request);


static uint8_t modbus_response[MODBUS_FRAME_MAX];
static size_t modbus_response_len = 0U;

static uint8_t response_buffer[MODBUS_FRAME_MAX];

/* Decoded bit values: enough for the protocol's 2000-bit read limit */
#define MODBUS_MAX_BITS 2000U


static uint8_t modbus_exception_code = 0U;

/* ---------------- MQTT configuration ---------------- */

#define MQTT_BROKER_HOST       "broker.emqx.io"
#define MQTT_BROKER_PORT       "1883"
#define MQTT_USERNAME          "1234"
#define MQTT_PASSWORD          "abcd"
#define MQTT_CLIENT_ID         "l511_device1"
#define MQTT_QOS                QOS1
#define MQTT_TOPIC             "l511/device1/data"
#define MQTT_CMD_TOPIC         "l511/device01/cmd"

#define MQTT_LOCAL_CID         mbtk_cid_index_2
#define MQTT_LOCAL_CID_PDN     mbtk_data_call_v4v6

/* ---------------- UART configuration ---------------- */

//#define UART_RESPONSE_MAX      128
  /* 400 x 20 ms = 8 seconds */

mbtk_taskref mbtk_test_task_ref;

static mbtk_pin_num_enum tx_pin = mbtk_pin_18;
static mbtk_pin_num_enum rx_pin = mbtk_pin_17;

static OL_UART_DCB uart_dcb = {0};
static mqtt_client_t *mqtt_client = NULL;

/* Function declarations */
void init_cdc_uart(void);
void cdc_uart_printf(const char *fmt, ...);
void app_open_task(void *argv);

int gpio_config(void);// gpio configuration function
int uart_config(void);// uart configuration function
int app_network_init(void);// netowrk registration and data call function
int app_mqtt_init(void);// mqtt intialization function

static void mqtt_message_callback(
    void *client,
    message_data_t *msg); // used to print the messege received from the mqtt broker to the uart terminal and validate crc
static int modbus_run_transaction(
    const uint8_t *request,
    size_t request_len,
    size_t expected_normal_response_len); // used to send the modbus request to the uart terminal and receive the response
static int modbus_response_validation(const uint8_t *request,
    size_t request_len,
    uint16_t quantity); // used to validate the modbus response received from the uart terminal

static int mqtt_publish_modbus_response(
    mqtt_client_t *client,
    uint32_t packet_id,
    const uint8_t *response,
    size_t response_len);
 // used to publish the modbus response to the mqtt broker
static int app_get_modbus_quantity(
    const uint8_t *payload,
    size_t len,
    uint16_t *quantity); // used to get the quantity of the modbus request from the mqtt payload
    
/* =====================================================
   APPLICATION INITIALIZATION
   ===================================================== */

void user_app_init(open_api_table *api_table)
{
    int ret;

    mbtk_api_init(api_table);
    init_cdc_uart();

    cdc_uart_printf("\r\n=== L511 MQTT-UART BRIDGE BOOT ===\r\n");

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

    ret = ol_os_task_creat(
        &mbtk_test_task_ref,
        NULL,
        50 * 1024,
        220,
        "mqtt_modbus",
        app_open_task,
        NULL
    );

    /*
     * Check ret against the task-creation success code
     * documented by your SDK.
     */
    cdc_uart_printf("[APP] Task creation return=%d\r\n", ret);
}
void app_open_task(void *argv)
{
    int ret;

    (void)argv;

    cdc_uart_printf("[APP] Initializing cellular network...\r\n");

    ret = app_network_init();

    if (ret != mbtk_data_call_ok)
    {
        cdc_uart_printf(
            "[APP] Network initialization failed: %d\r\n",
            ret
        );
        return;
    }

    cdc_uart_printf("[APP] Network ready\r\n");

    ret = app_mqtt_init();

    if (ret != MQTT_SUCCESS_ERROR)
    {
        cdc_uart_printf(
            "[APP] MQTT initialization failed: %d\r\n",
            ret
        );
        return;
    }

    cdc_uart_printf("[APP] MQTT connected and subscribed\r\n");

    /*
     * Keep this task alive if required by your SDK's task model.
     * MQTT message dispatch must be handled by the SDK's client
     * processing mechanism.
     */
  while (1)
{
    modbus_queued_request_t queued_request;
    int queue_status;

    queue_status = modbus_queue_pop(&queued_request);

    if (queue_status == 1)
    {
        modbus_process_queued_request(&queued_request);
    }
    else if (queue_status < 0)
    {
        cdc_uart_printf(
            "[QUEUE] Dequeue error: %d\r\n",
            queue_status);

        ol_os_task_sleep(1);
    }
    else
    {
        /* No pending request. Yield while waiting. */
        ol_os_task_sleep(1);
    }
}
}




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
   UART CONFIGURATION: 115200, 8-N-1
   ===================================================== */

int uart_config(void)
{
    int err;

    err = ol_Uart_GetDcb(OL_UART_PORT_STUART, &uart_dcb);

    if (err != OL_UART_RC_OK)
    {
        cdc_uart_printf("[ERROR] ol_Uart_GetDcb: %d\r\n", err);
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
        cdc_uart_printf("[ERROR] ol_Uart_SetDcb: %d\r\n", err);
        return -1;
    }

    err = ol_Uart_Open(OL_UART_PORT_STUART);

    if (err != OL_UART_RC_OK)
    {
        cdc_uart_printf("[ERROR] ol_Uart_Open: %d\r\n", err);
        return -1;
    }

    cdc_uart_printf("[UART] STUART ready: 9600 8-N-1\r\n");

    return 0;
}


/*
 * Establish cellular registration and packet-data connection.
 *
 * Returns:
 *   mbtk_data_call_ok on success
 *   Modem SDK error code on failure
 */
int app_network_init(void)
{
    int ret;
    unsigned int attempt = 0;

    while (1)
    {
        attempt++;

        cdc_uart_printf(
            "\r\n========== NETWORK ATTEMPT %u ==========\r\n",
            attempt
        );

        /* Step 1: Cellular network registration */
        cdc_uart_printf(
            "[NETWORK] Waiting for cellular registration...\r\n"
        );

        ret = ol_wait_network_regist(120);

        cdc_uart_printf(
            "[NETWORK] Registration return=%d\r\n",
            ret
        );

        if (ret != mbtk_data_call_ok)
        {
            cdc_uart_printf(
                "[NETWORK] Registration failed, ret=%d\r\n",
                ret
            );

            cdc_uart_printf(
                "[NETWORK] Retrying in 5 seconds...\r\n"
            );

            ol_os_task_sleep(5 * 200);
            continue;
        }

        cdc_uart_printf(
            "[NETWORK] Cellular registration succeeded\r\n"
        );

        /* Step 2: Establish packet-data connection */
        cdc_uart_printf(
            "[NETWORK] Starting data call, APN=%s\r\n",
            "IOT.COM"
        );

        ret = ol_data_call_start(
            MQTT_LOCAL_CID,
            MQTT_LOCAL_CID_PDN,
            "IOT.COM",
            NULL,
            NULL,
            0
        );

        cdc_uart_printf(
            "[NETWORK] Data-call return=%d\r\n",
            ret
        );

        if (ret != mbtk_data_call_ok)
        {
            cdc_uart_printf(
                "[NETWORK] Data-call failed, ret=%d\r\n",
                ret
            );

            cdc_uart_printf(
                "[NETWORK] Retrying in 5 seconds...\r\n"
            );

            ol_os_task_sleep(5 * 200);
            continue;
        }

        cdc_uart_printf(
            "[NETWORK] Packet-data connection established\r\n"
        );

        return mbtk_data_call_ok;
    }
}


int app_mqtt_init(void)
{
    int ret;

    /* Step 1: Acquire an MQTT client */
    mqtt_client = ol_mqtt_lease();

    if (mqtt_client == NULL)
    {
        return -1;
    }

    /* Step 2: Configure the MQTT broker */
    ol_mqtt_set_host(mqtt_client, (char *)MQTT_BROKER_HOST);
    ol_mqtt_set_port(mqtt_client, (char *)MQTT_BROKER_PORT);
    ol_mqtt_set_client_id(mqtt_client, (char *)MQTT_CLIENT_ID);
    ol_mqtt_set_user_name(mqtt_client, (char *)MQTT_USERNAME);
    ol_mqtt_set_password(mqtt_client, (char *)MQTT_PASSWORD);

    /* Step 3: Connect to the MQTT broker */
    ret = ol_mqtt_connect(mqtt_client);

    if (ret != MQTT_SUCCESS_ERROR)
    {
        return ret;
    }

    /* Step 4: Subscribe to the command topic */
    ret = ol_mqtt_subscribe(
        mqtt_client,
        MQTT_CMD_TOPIC,
        MQTT_QOS,
        mqtt_message_callback
    );

    if (ret != MQTT_SUCCESS_ERROR)
    {
        return ret;
    }

    return MQTT_SUCCESS_ERROR;
}





static void mqtt_message_callback(
    void *client,
    message_data_t *msg)
{
    mqtt_message_t *incoming;
    const uint8_t *payload;
    size_t payload_len;
    int ret;

    if (client == NULL ||
        msg == NULL ||
        msg->message == NULL)
    {
        cdc_uart_printf(
            "[MQTT RX] Invalid MQTT message\r\n");
        return;
    }

    if (msg->topic_name == NULL ||
        strcmp(msg->topic_name, MQTT_CMD_TOPIC) != 0)
    {
        return;
    }

    incoming = msg->message;

    if (incoming->payload == NULL ||
        incoming->payloadlen <= 0)
    {
        cdc_uart_printf(
            "[MQTT RX] Empty message\r\n");
        return;
    }

    payload = (const uint8_t *)incoming->payload;
    payload_len = (size_t)incoming->payloadlen;

    /*
     * Payload layout:
     * [0..3] packet ID, uint32 big-endian
     * [4..]  complete Modbus RTU request including CRC
     */
    ret = modbus_queue_push_payload(
        payload,
        payload_len);

    if (ret != 0)
    {
        cdc_uart_printf(
            "[MQTT RX] Queue rejected request: %d, "
            "payload length=%u\r\n",
            ret,
            (unsigned int)payload_len);
        return;
    }

    cdc_uart_printf(
        "[MQTT RX] Request queued, payload length=%u\r\n",
        (unsigned int)payload_len);
}





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

     ol_Uart_Write(
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



/*
static int modbus_response_validation(const uint8_t *request,
    size_t request_len,
    uint16_t quantity)
{
    int result;
    size_t i;
    uint8_t function;

    function = request[1];

    if (function != 0x01U && function != 0x02U)
    {
        cdc_uart_printf(
            "[MODBUS] This validation function currently handles FC01/FC02 only\r\n");
        return -1;
    }

    result = modbus_parse_response(
        request,
        request_len,
        modbus_response,
        modbus_response_len,
        NULL,
        0U,
        modbus_discrete_inputs,
        (size_t)quantity,
        &modbus_exception_code);

    if (result == 0)
    {
        cdc_uart_printf("[FC%02X] Bit values: ",
                        (unsigned int)function);

        for (i = 0U; i < (size_t)quantity; i++)
        {
            cdc_uart_printf("%u ",
                (unsigned int)modbus_discrete_inputs[i]);
        }

        cdc_uart_printf("\r\n");
        return 0;
    }

    if (result == 1)
    {
        cdc_uart_printf(
            "[MODBUS] Exception response: 0x%02X\r\n",
            (unsigned int)modbus_exception_code);
        return 1;
    }

    cdc_uart_printf("[MODBUS] Parser error: %d\r\n", result);
    return -5;
}
*/



static int mqtt_publish_modbus_response(
    mqtt_client_t *client,
    uint32_t packet_id,
    const uint8_t *response,
    size_t response_len);

/* =====================================================
   MQTT RESPONSE: PACKET ID + MODBUS RTU FRAME
   ===================================================== */

static uint8_t mqtt_response_payload[MQTT_MAX_PAYLOAD];

static int mqtt_publish_modbus_response(
    mqtt_client_t *client,
    uint32_t packet_id,
    const uint8_t *response,
    size_t response_len)
{
    mqtt_message_t message;
    size_t total_len;
    int ret;

    if (client == NULL ||
        response == NULL ||
        response_len < 5U ||
        response_len > MODBUS_FRAME_MAX ||
        packet_id == 0U)
    {
        cdc_uart_printf(
            "[MQTT] Invalid response publish arguments\r\n");
        return -1;
    }

    total_len = MQTT_PACKET_ID_SIZE + response_len;

    /* Encode packet ID as unsigned 32-bit big-endian. */
    mqtt_response_payload[0] =
        (uint8_t)(packet_id >> 24);

    mqtt_response_payload[1] =
        (uint8_t)(packet_id >> 16);

    mqtt_response_payload[2] =
        (uint8_t)(packet_id >> 8);

    mqtt_response_payload[3] =
        (uint8_t)packet_id;

    /* Append the raw RTU response, including its CRC. */
    memcpy(
        &mqtt_response_payload[MQTT_PACKET_ID_SIZE],
        response,
        response_len);

    memset(&message, 0, sizeof(message));

    message.qos = QOS1;
    message.retained = 0;
    message.payload = (void *)mqtt_response_payload;
    message.payloadlen = (int)total_len;

    ret = ol_mqtt_publish(
        client,
        MQTT_TOPIC,
        &message);

    cdc_uart_printf(
        "[MQTT TX] Packet ID=%lu, payload length=%u, "
        "publish return=%d\r\n",
        (unsigned long)packet_id,
        (unsigned int)total_len,
        ret);

    return ret;
}


/*
static int app_get_modbus_quantity(
    const uint8_t *payload,
    size_t len,
    uint16_t *quantity)
{
    uint8_t function;

    if (payload == NULL || quantity == NULL || len < 8U)
    {
        return -1;
    }

    *quantity = 0U;
    function = payload[1];

    switch (function)
    {
       // case 0x01U: /* Read Coils */
    //    case 0x02U: /* Read Discrete Inputs */
  //      case 0x03U: /* Read Holding Registers */
      //  case 0x04U: /* Read Input Registers */
        //case 0x0FU: /* Write Multiple Coils */
        //case 0x10U: /* Write Multiple Registers */
          //  *quantity = (uint16_t)(
            //    ((uint16_t)payload[4] << 8) |
              //  (uint16_t)payload[5]);
           // break;

      //  case 0x05U: /* Write Single Coil */
        //case 0x06U: /* Write Single Register */
          //  *quantity = 1U;
           // break;

      //  default:
        //    return -1;
    //}

    ///return (*quantity != 0U) ? 0 : -1;
//}



/* =====================================================
   QUEUE PRODUCER: MQTT PAYLOAD -> QUEUE
   ===================================================== */

static int modbus_queue_push_payload(
    const uint8_t *payload,
    size_t payload_len)
{
    uint32_t head;
    uint32_t tail;
    uint32_t slot;
    uint32_t packet_id;
    size_t frame_len;
    size_t i;
    modbus_queued_request_t *entry;

    if (payload == NULL)
    {
        return -1;
    }

    /*
     * Minimum payload:
     * 4-byte packet ID + minimum 4-byte RTU frame.
     */
    if (payload_len < 8U || payload_len > MQTT_MAX_PAYLOAD)
    {
        return -2;
    }

    frame_len = payload_len - MQTT_PACKET_ID_SIZE;

    if (frame_len > MODBUS_FRAME_MAX)
    {
        return -2;
    }

    /* Packet ID is unsigned 32-bit, big-endian. */
    packet_id =
        ((uint32_t)payload[0] << 24) |
        ((uint32_t)payload[1] << 16) |
        ((uint32_t)payload[2] << 8)  |
        ((uint32_t)payload[3]);

    if (packet_id == 0U)
    {
        return -3;
    }

    head = modbus_queue_head;
    tail = modbus_queue_tail;

    /* Queue full: do not overwrite an unread request. */
    if ((uint32_t)(head - tail) >= MODBUS_QUEUE_DEPTH)
    {
        return -4;
    }

    slot = head % MODBUS_QUEUE_DEPTH;
    entry = &modbus_request_queue[slot];

    entry->packet_id = packet_id;
    entry->frame_len = frame_len;

    for (i = 0U; i < frame_len; i++)
    {
        entry->frame[i] = payload[MQTT_PACKET_ID_SIZE + i];
    }

    /*
     * Publish the new head only after the complete slot
     * has been written.
     */
    MODBUS_QUEUE_BARRIER();
    modbus_queue_head = head + 1U;

    return 0;
}


/* =====================================================
   QUEUE CONSUMER: QUEUE -> LOCAL REQUEST
   ===================================================== */

static int modbus_queue_pop(
    modbus_queued_request_t *request)
{
    uint32_t head;
    uint32_t tail;
    uint32_t slot;
    size_t i;

    if (request == NULL)
    {
        return -1;
    }

    tail = modbus_queue_tail;
    head = modbus_queue_head;

    if (tail == head)
    {
        return 0; /* Queue empty. */
    }

    /*
     * Observe the producer's completed slot write before
     * copying the request.
     */
    MODBUS_QUEUE_BARRIER();

    slot = tail % MODBUS_QUEUE_DEPTH;

    request->packet_id =
        modbus_request_queue[slot].packet_id;

    request->frame_len =
        modbus_request_queue[slot].frame_len;

    if (request->frame_len > MODBUS_FRAME_MAX)
    {
        /* Defensive recovery from corrupted queue metadata. */
        modbus_queue_tail = tail + 1U;
        return -2;
    }

    for (i = 0U; i < request->frame_len; i++)
    {
        request->frame[i] =
            modbus_request_queue[slot].frame[i];
    }

    /*
     * Release the slot only after copying it locally.
     */
    MODBUS_QUEUE_BARRIER();
    modbus_queue_tail = tail + 1U;

    return 1;
}



/* =====================================================
   QUEUED MODBUS REQUEST PROCESSOR
   ===================================================== */

static void modbus_process_queued_request(
    const modbus_queued_request_t *request)
{
    size_t expected_response_len = 0U;
    uint16_t quantity = 0U;
    uint16_t register_values[125U];
    uint8_t bit_values[MODBUS_MAX_BITS];
    int transaction_status;
    int parse_result;
    int ret;

    if (request == NULL ||
        request->frame_len < 4U ||
        request->frame_len > MODBUS_FRAME_MAX)
    {
        cdc_uart_printf(
            "[MODBUS] Invalid queued request\r\n");
        return;
    }

    cdc_uart_printf(
        "\r\n[QUEUE] Processing packet ID=%lu\r\n",
        (unsigned long)request->packet_id);

    /* Step 1: Validate request CRC. */
    ret = modbus_validate_crc(
        request->frame,
        request->frame_len);

    if (ret != 0)
    {
        cdc_uart_printf(
            "[MODBUS] Request CRC validation failed\r\n");
        return;
    }

    cdc_uart_printf("[MODBUS CRC] PASS\r\n");

    /* Step 2: Determine the expected response size. */
    ret = modbus_expected_response_length(
        request->frame,
        request->frame_len,
        &expected_response_len,
        &quantity);

    if (ret != 0 ||
        expected_response_len < 5U ||
        expected_response_len > MODBUS_FRAME_MAX)
    {
        cdc_uart_printf(
            "[MODBUS] Invalid request or response length\r\n");
        return;
    }

    /* quantity is retained for compatibility with your API. */
    (void)quantity;

    /* Step 3: Execute the existing UART transaction. */
    transaction_status = modbus_run_transaction(
        request->frame,
        request->frame_len,
        expected_response_len);

    if (transaction_status != 0)
    {
        cdc_uart_printf(
            "[MODBUS] Transaction failed: %d\r\n",
            transaction_status);
        return;
    }

    /* Step 4: Parse and validate the RTU response. */
    parse_result = modbus_parse_response(
        request->frame,
        request->frame_len,
        modbus_response,
        modbus_response_len,
        register_values,
        sizeof(register_values) / sizeof(register_values[0]),
        bit_values,
        sizeof(bit_values) / sizeof(bit_values[0]),
        &modbus_exception_code);

    if (parse_result < 0)
    {
        cdc_uart_printf(
            "[MODBUS] Response validation failed: %d\r\n",
            parse_result);
        return;
    }

    if (parse_result == 1)
    {
        cdc_uart_printf(
            "[MODBUS] Exception response: 0x%02X\r\n",
            (unsigned int)modbus_exception_code);
    }
    else
    {
        cdc_uart_printf(
            "[MODBUS] Response valid\r\n");
    }

    /*
     * Step 5: Publish packet ID + raw RTU response.
     * This includes the response CRC.
     */
    ret = mqtt_publish_modbus_response(
        mqtt_client,
        request->packet_id,
        modbus_response,
        modbus_response_len);

    if (ret != MQTT_SUCCESS_ERROR)
    {
        cdc_uart_printf(
            "[MQTT] Response publish failed: %d\r\n",
            ret);
        return;
    }

    cdc_uart_printf(
        "[QUEUE] Completed packet ID=%lu\r\n",
        (unsigned long)request->packet_id);
}
