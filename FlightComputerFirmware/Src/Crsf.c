#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stm32f3xx_hal.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "Ticks.h"
#include "Crsf.h"
#include "Serial3.h"

#define CRSF_CHANNEL_VALUE_MIN  172
#define CRSF_CHANNEL_VALUE_1000 191
#define CRSF_CHANNEL_VALUE_MID  992
#define CRSF_CHANNEL_VALUE_2000 1792
#define CRSF_CHANNEL_VALUE_MAX  1811
#define CRSF_CHANNEL_VALUE_SPAN (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN)
#define CRSF_MAX_PACKET_LEN 64

enum CRSF_FRAME
{
  CRSF_FRAME_GPS_PAYLOAD_SIZE = 15,
  CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE = 8,
  CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE = 10,
  CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE = 22, // 11 bits per channel * 16 channels = 22 bytes.
  CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE = 6,
};

enum CRSF_FRAMETYPE
{
  CRSF_FRAMETYPE_GPS = 0x02,
  CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
  CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
  CRSF_FRAMETYPE_OPENTX_SYNC = 0x10,
  CRSF_FRAMETYPE_RADIO_ID = 0x3A,
  CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
  CRSF_FRAMETYPE_ATTITUDE = 0x1E,
  CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
  // Extended Header Frames, range: 0x28 to 0x96
  CRSF_FRAMETYPE_DEVICE_PING = 0x28,
  CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
  CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
  CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
  CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
  CRSF_FRAMETYPE_COMMAND = 0x32,
  // MSP commands
  CRSF_FRAMETYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
  CRSF_FRAMETYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
  CRSF_FRAMETYPE_MSP_WRITE = 0x7C, // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
};

enum CRSF_ADDRESS
{
  CRSF_ADDRESS_BROADCAST = 0x00,
  CRSF_ADDRESS_USB = 0x10,
  CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
  CRSF_ADDRESS_RESERVED1 = 0x8A,
  CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
  CRSF_ADDRESS_GPS = 0xC2,
  CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
  CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
  CRSF_ADDRESS_RESERVED2 = 0xCA,
  CRSF_ADDRESS_RACE_TAG = 0xCC,
  CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
  CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
  CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,
};

typedef struct crsf_channels_s
{
  uint16_t ch0 : 11;
  uint16_t ch1 : 11;
  uint16_t ch2 : 11;
  uint16_t ch3 : 11;
  uint16_t ch4 : 11;
  uint16_t ch5 : 11;
  uint16_t ch6 : 11;
  uint16_t ch7 : 11;
  uint16_t ch8 : 11;
  uint16_t ch9 : 11;
  uint16_t ch10 : 11;
  uint16_t ch11 : 11;
  uint16_t ch12 : 11;
  uint16_t ch13 : 11;
  uint16_t ch14 : 11;
  uint16_t ch15 : 11;
} __attribute__((packed)) CrsfChannelsPacked_t;

static uint8_t crc8_lut[256];

static QueueHandle_t crsf_status_queue;

static CrsfStatus_t crsf_status = {0};

static SerialInterface_t* serial_interface;

#define HEADER_SIZE         1
#define MESSAGE_SIZE_SIZE   1
#define MESSAGE_TYPE_SIZE   1
#define CRC_SIZE            1

enum PARSER_STATE
{
  PARSER_STATE_HEADER,
  PARSER_STATE_MESSAGE_SIZE,
  PARSER_STATE_MESSAGE_TYPE,
  PARSER_STATE_PAYLOAD,
  PARSER_STATE_CRC,
};

static enum PARSER_STATE parser_state = PARSER_STATE_HEADER;
static uint32_t working_index = 0;
static uint32_t working_segment_size = HEADER_SIZE;

static uint8_t process_buffer[CRSF_MAX_PACKET_LEN];

#define CRSF_PERIOD   (20 / portTICK_PERIOD_MS)

static void Crc8Init(const uint8_t poly);
static uint8_t Crc8Calc(const uint8_t *data, uint8_t size);
static void ParseCrsfPackets(void);
void CrsfTask(void* arg);

static const char* TAG = "CSRF";

void Crsf_Init(void)
{
  Crc8Init(0xd5);
  
  crsf_status_queue = xQueueCreate(1, sizeof(CrsfStatus_t));
  
  serial_interface = Serial3_GetInterface();
  serial_interface->init(0, 0);
  
  xTaskCreate(CrsfTask, TAG, 500, NULL, 0, NULL);
}

bool Crsf_GetStatus(CrsfStatus_t* const status)
{
  return (xQueuePeek(crsf_status_queue, status, 0) == pdTRUE);
}

void CrsfTask(void* arg)
{
  while (true)
  {
    xEventGroupWaitBits(*serial_interface->serial_events, SERIAL_INTERFACE_EVENT_NEW_DATA_RX, false, false, CRSF_PERIOD);

    if (xEventGroupGetBits(*serial_interface->serial_events) & SERIAL_INTERFACE_EVENT_NEW_DATA_RX)
    {
      xEventGroupClearBits(*serial_interface->serial_events, SERIAL_INTERFACE_EVENT_NEW_DATA_RX);
    }

    // Parse the potentially new data
    ParseCrsfPackets();
  }
}

// 0xC8 [packet len] [packet type] [data] [crc]
static void ParseCrsfPackets(void)
{
  uint32_t buffer_count;
  uint8_t working_byte;

  buffer_count = QueueBuffer_Count(serial_interface->rx_queue);

  // Iterate through the buffer to parse the message out
  while ((working_index < buffer_count) && (buffer_count - working_index) >= working_segment_size)
  {
    switch (parser_state)
    {
      // Header
      case PARSER_STATE_HEADER:
        if (QueueBuffer_Get(serial_interface->rx_queue, &working_byte))
        {
          if (working_byte == 0xc8)
          {
            printf("H\r\n");
            parser_state = PARSER_STATE_MESSAGE_SIZE;
            working_index = 0;
            continue;
          }
        }

        working_index = 0;
        working_segment_size = HEADER_SIZE;
        break;

      // Message size
      case PARSER_STATE_MESSAGE_SIZE:
        working_index++;
        working_segment_size = 1;
        parser_state = PARSER_STATE_MESSAGE_TYPE;
        break;
        
      // Message type
      case PARSER_STATE_MESSAGE_TYPE:
        if (working_byte != CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
        {
          parser_state = PARSER_STATE_HEADER;
          working_segment_size = HEADER_SIZE;
          working_index = 0;
          continue;
        }
      
        working_index++;
        working_segment_size = CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE;
        parser_state = PARSER_STATE_PAYLOAD;
        break;

      // Full message content
      case PARSER_STATE_PAYLOAD:
        working_index += CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE;
        working_segment_size = CRC_SIZE;
        parser_state = PARSER_STATE_CRC;
        break;

      // CRC16
      case PARSER_STATE_CRC:
        // Fetch the suspected message as a contingous block of memory
        QueueBuffer_PeekBuffer(serial_interface->rx_queue, 0, process_buffer, working_index + CRC_SIZE);

        // Verify checksum
        if (Crc8Calc(process_buffer, working_index) == process_buffer[working_index])
        {
          printf("packet OK\r\n");
          // Remove the sucessfully processed data from the queue
          QueueBuffer_Dequeue(serial_interface->rx_queue, working_index + CRC_SIZE);
        }

        working_index = 0;
        working_segment_size = HEADER_SIZE;
        parser_state = PARSER_STATE_HEADER;
        break;
    }

    buffer_count = QueueBuffer_Count(serial_interface->rx_queue);
  }
}

static void Crc8Init(const uint8_t poly)
{
  for (int idx=0; idx<256; ++idx)
  {
    uint8_t crc = idx;
    for (int shift = 0; shift<8; ++shift)
    {
      crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
    }
    crc8_lut[idx] = crc & 0xff;
  }
}

static uint8_t Crc8Calc(const uint8_t *data, uint8_t size)
{
  uint8_t crc = 0;
  while (size--)
  {
    crc = crc8_lut[crc ^ *data++];
  }
  return crc;
}