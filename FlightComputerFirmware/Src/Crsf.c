#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stm32f4xx_hal.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "Ticks.h"
#include "Crsf.h"
#include "Serial3.h"
#include "MathX.h"

#define CRSF_CHANNEL_VALUE_MIN  172
#define CRSF_CHANNEL_VALUE_MAX  1811
#define CRSF_CHANNEL_VALUE_SPAN (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN)
#define CRSF_MAX_PACKET_LEN 64
#define CRSF_HEADER 0xc8

enum CRSF_PAYLOAD_SIZE
{
  CRSF_PAYLOAD_SIZE_GPS = 15,
  CRSF_PAYLOAD_SIZE_BATTERY = 8,
  CRSF_PAYLOAD_SIZE_LINK_STATISTICS = 10,
  CRSF_PAYLOAD_SIZE_RC_CHANNELS = 22,
  CRSF_PAYLOAD_SIZE_ATTITUDE = 6,
};

enum CRSF_PACKET_TYPE
{
  CRSF_PACKET_TYPE_GPS = 0x02,
  CRSF_PACKET_TYPE_BATTERY_SENSOR = 0x08,
  CRSF_PACKET_TYPE_LINK_STATISTICS = 0x14,
  CRSF_PACKET_TYPE_OPENTX_SYNC = 0x10,
  CRSF_PACKET_TYPE_RADIO_ID = 0x3A,
  CRSF_PACKET_TYPE_RC_CHANNELS_PACKED = 0x16,
  CRSF_PACKET_TYPE_ATTITUDE = 0x1E,
  CRSF_PACKET_TYPE_FLIGHT_MODE = 0x21,
  // Extended Header Frames, range: 0x28 to 0x96
  CRSF_PACKET_TYPE_DEVICE_PING = 0x28,
  CRSF_PACKET_TYPE_DEVICE_INFO = 0x29,
  CRSF_PACKET_TYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
  CRSF_PACKET_TYPE_PARAMETER_READ = 0x2C,
  CRSF_PACKET_TYPE_PARAMETER_WRITE = 0x2D,
  CRSF_PACKET_TYPE_COMMAND = 0x32,
  // MSP commands
  CRSF_PACKET_TYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
  CRSF_PACKET_TYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
  CRSF_PACKET_TYPE_MSP_WRITE = 0x7C, // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
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

static uint8_t crc8_lut[256];

static QueueHandle_t crsf_status_queue;

static CrsfStatus_t crsf_status = {0};

static SerialInterface_t* serial_interface;

#define HEADER_SIZE           1
#define PACKET_SIZE_SIZE      1
#define PACKET_TYPE_SIZE      1
#define PACKET_SIZE_TYPE_SIZE 2
#define CRC_SIZE              1

enum PARSER_STATE
{
  PARSER_STATE_HEADER,
  PARSER_STATE_SIZE_TYPE,
  PARSER_STATE_PAYLOAD,
  PARSER_STATE_CRC,
};

static enum PARSER_STATE parser_state = PARSER_STATE_HEADER;
static uint32_t working_index = 0;
static uint32_t working_segment_size = HEADER_SIZE;

static uint8_t process_buffer[CRSF_MAX_PACKET_LEN];

typedef struct
{
  uint8_t packet_type;
  uint32_t packet_size;
  bool (*processor)(const uint8_t* data, const uint32_t size);
} CsrfPacketDescriptor_t;

static bool ProcessChannelData(const uint8_t* data, const uint32_t size);
static bool ProcessLinkStatistics(const uint8_t* data, const uint32_t size);

#define CSRF_PACKET_DESCRIPTOR_COUNT  2
static const CsrfPacketDescriptor_t csrf_packet_descriptors[CSRF_PACKET_DESCRIPTOR_COUNT] =
{
  {CRSF_PACKET_TYPE_RC_CHANNELS_PACKED, CRSF_PAYLOAD_SIZE_RC_CHANNELS, ProcessChannelData},
  {CRSF_PACKET_TYPE_LINK_STATISTICS, CRSF_PAYLOAD_SIZE_LINK_STATISTICS, ProcessLinkStatistics},
};

static CsrfPacketDescriptor_t* working_descriptor = NULL;

#define CRSF_PERIOD   (20 / portTICK_PERIOD_MS)

static void Crc8Init(const uint8_t poly);
static uint8_t Crc8Calc(const uint8_t *data, uint8_t size);
static void ParseCrsfPackets(void);
static void CrsfTask(void* arg);
static CsrfPacketDescriptor_t* FindCrsfDescriptor(const enum CRSF_PACKET_TYPE packet_type);

static const char* TAG = "CSRF";

void Crsf_Init(void)
{
  Crc8Init(0xd5);

  crsf_status_queue = xQueueCreate(1, sizeof(CrsfStatus_t));

  serial_interface = Serial3_GetInterface();
  serial_interface->init(0, 0);

  xTaskCreate(CrsfTask, TAG, 800, NULL, 0, NULL);
}

bool Crsf_GetStatus(CrsfStatus_t* const status)
{
  return (xQueuePeek(crsf_status_queue, status, 0) == pdTRUE);
}

static bool ProcessChannelData(const uint8_t* data, const uint32_t size)
{
  uint32_t raw_channels[CRSF_CHANNEL_COUNT];
  uint32_t i;

  // Decode channel data
  raw_channels[0] = (data[0] | data[1] << 8) & 0x07FF;
  raw_channels[1] = (data[1]  >> 3 | data[2] << 5) & 0x07FF;
  raw_channels[2] = (data[2]>> 6 | data[3] << 2 | data[4] << 10) & 0x07FF;
  raw_channels[3] = (data[4]>> 1 | data[5] << 7) & 0x07FF;
  raw_channels[4] = (data[5]>> 4 | data[6] << 4) & 0x07FF;
  raw_channels[5] = (data[6]>> 7 | data[7] << 1 | data[8] << 9) & 0x07FF;
  raw_channels[6] = (data[8]>> 2 | data[9] << 6) & 0x07FF;
  raw_channels[7] = (data[9] >> 5 | data[10] << 3) & 0x07FF;
  raw_channels[8] = (data[11]| data[12] << 8) & 0x07FF;
  raw_channels[9] = (data[12] >> 3 | data[13] << 5) & 0x07FF;
  raw_channels[10] = (data[13] >> 6 | data[14] << 2 | data[15] << 10) & 0x07FF;
  raw_channels[11] = (data[15] >> 1 | data[16] << 7) & 0x07FF;
  raw_channels[12] = (data[16] >> 4 | data[17] << 4) & 0x07FF;
  raw_channels[13] = (data[17] >> 7 | data[18] << 1 | data[19] << 9) & 0x07FF;
  raw_channels[14] = (data[19] >> 2 | data[20] << 6) & 0x07FF;
  raw_channels[15] = (data[20] >> 5 | data[21] << 3) & 0x07FF;

  crsf_status.channel_data.timestamp = Ticks_Now();

  for (i = 0; i < CRSF_CHANNEL_COUNT; i++)
  {
    raw_channels[i] = MathX_Constrain(raw_channels[i], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
    crsf_status.channel_data.channels[i] = MathX_MapF((float)raw_channels[i], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, -1000.0f, 1000.0f);
  }

  xQueueOverwrite(crsf_status_queue, &crsf_status);

  return true;
}

static bool ProcessLinkStatistics(const uint8_t* data, const uint32_t size)
{
  crsf_status.link_statistics.timestamp = Ticks_Now();
  crsf_status.link_statistics.uplink_rssi_1 = data[0];
  crsf_status.link_statistics.uplink_rssi_2  = data[1];
  crsf_status.link_statistics.uplink_link_quality = data[2];
  crsf_status.link_statistics.uplink_snr = data[3];
  crsf_status.link_statistics.active_antenna = data[4];
  crsf_status.link_statistics.rf_mode = data[5];
  crsf_status.link_statistics.uplink_tx_power = data[6];
  crsf_status.link_statistics.downlink_rssi = data[7];
  crsf_status.link_statistics.downlink_link_quality = data[8];
  crsf_status.link_statistics.downlink_snr = data[9];

  xQueueOverwrite(crsf_status_queue, &crsf_status);

  return true;
}

static CsrfPacketDescriptor_t* FindCrsfDescriptor(const enum CRSF_PACKET_TYPE packet_type)
{
  uint32_t i;

  for (i = 0; i < CSRF_PACKET_DESCRIPTOR_COUNT; i++)
  {
    if (csrf_packet_descriptors[i].packet_type == packet_type)
    {
      return (CsrfPacketDescriptor_t*)&csrf_packet_descriptors[i];
    }
  }

  return NULL;
}

static void CrsfTask(void* arg)
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
  uint8_t packet_size;
  uint8_t packet_type;

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
          if (working_byte == CRSF_HEADER)
          {
            parser_state = PARSER_STATE_SIZE_TYPE;
            working_segment_size = PACKET_SIZE_TYPE_SIZE;
            working_index = 0;
            continue;
          }
        }

        working_index = 0;
        working_segment_size = HEADER_SIZE;
        break;

      // Packet size type
      case PARSER_STATE_SIZE_TYPE:
        QueueBuffer_Peek(serial_interface->rx_queue, working_index++, &packet_size);
        QueueBuffer_Peek(serial_interface->rx_queue, working_index++, &packet_type);

        working_descriptor = FindCrsfDescriptor(packet_type);

        if (working_descriptor == NULL
          || packet_size != working_descriptor->packet_size + PACKET_SIZE_TYPE_SIZE)
        {
          parser_state = PARSER_STATE_HEADER;
          working_segment_size = HEADER_SIZE;
          working_index = 0;
          continue;
        }

        parser_state = PARSER_STATE_PAYLOAD;
        working_segment_size = working_descriptor->packet_size;
        break;

      // Full packet content
      case PARSER_STATE_PAYLOAD:
        working_index += working_descriptor->packet_size;
        working_segment_size = CRC_SIZE;
        parser_state = PARSER_STATE_CRC;
        break;

      // CRC
      case PARSER_STATE_CRC:
        // Fetch the suspected packet as a contingous block of memory
        QueueBuffer_PeekBuffer(serial_interface->rx_queue, 0, process_buffer, working_index + CRC_SIZE);

        // Verify checksum
        if (Crc8Calc(process_buffer + PACKET_SIZE_SIZE, working_descriptor->packet_size + PACKET_TYPE_SIZE) == process_buffer[working_index])
        {
          if (working_descriptor->processor(process_buffer + PACKET_SIZE_TYPE_SIZE, working_index - PACKET_SIZE_TYPE_SIZE))
          {
            // Remove the sucessfully processed data from the queue
            QueueBuffer_Dequeue(serial_interface->rx_queue, working_index + CRC_SIZE);
          }
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
  for (int idx=0; idx < 256; ++idx)
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