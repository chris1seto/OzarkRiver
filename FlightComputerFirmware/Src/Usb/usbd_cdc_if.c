#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include "semphr.h"
#include "usbd_cdc_if.h"
#include "Usb.h"
#include "QueueBuffer.h"
#include "CrcCcitt.h"
#include "Radio.h"
#include "Config.h"
#include "FlashConfig.h"
#include "Helpers.h"
#include <math.h>

static int8_t TEMPLATE_Init(void);
static int8_t TEMPLATE_DeInit(void);
static int8_t TEMPLATE_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t TEMPLATE_Receive(uint8_t* pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef USBD_CDC_Template_fops = 
{
  TEMPLATE_Init,
  TEMPLATE_DeInit,
  TEMPLATE_Control,
  TEMPLATE_Receive
};

#define USB_BUFFER_SIZE		128
static uint8_t bufferTx[USB_BUFFER_SIZE];
static uint8_t bufferRx[USB_BUFFER_SIZE];

extern USBD_HandleTypeDef USBD_Device;

// Fake line encoding info so that the PC can decide it's properly configured us as a serial device
static uint32_t fakeBaud = 115200;
static uint8_t fakeParity = 0;
static uint8_t fakeFormat = 0;
static uint8_t fakeDatetype = 0;

#define PACKET_BUFFER_SIZE		500
static uint8_t packetQueueBuffer[PACKET_BUFFER_SIZE];
static QueueBuffer_t packetQueue;
static uint8_t xBuffer[100];

#define PARSER_STATE_HEADER		0
#define PARSER_STATE_COMMAND	1
#define PARSER_STATE_PAYLOAD	2
#define PARSER_STATE_CRC1		3
#define PARSER_STATE_CRC0		4

static uint8_t parserState = 0;
static uint32_t parserIndex = 0;
static uint16_t crcPacket;

struct CommandT
{
	uint8_t Ident;
	uint32_t Length;
	void(*Handler)(const uint8_t*);
};

static struct CommandT* workingCommand;

static uint8_t UsbCommandParse(void);
static void ProcessSettingsCommand(const uint8_t* pointer);
static void ProcessTestCommand(const uint8_t* pointer);
static void ProcessMessageCommand(const uint8_t* pointer);

// APRS settings
 struct __attribute__((__packed__)) SettingsTransportT
{
	uint8_t Callsign[6];
	uint8_t Path[7];
	uint8_t Ssid;
	uint8_t SymbolTable;
	uint8_t Symbol;
	uint32_t BeaconPeriod;
	uint8_t UseSmartBeacon;
	uint32_t SmartBeaconMinimumBeaconPeriod;
	float SmartBeaconHeadingTuning;
	float SmartBeaconSpeedTuning;
	float SmartBeaconWeightSpeed;
	float SmartBeaconWeightdSpeed;
	float SmartBeaconWeightdHeading;
};

// APRS message
struct __attribute__((__packed__)) MessageTransportT
{
	uint8_t DestinationCallsign[9];
	uint8_t MessageLength;
	uint8_t Message[67];
	uint32_t MessageNumber;
};

#define COMMAND_COUNT		3	
const struct CommandT commands[COMMAND_COUNT] = 
{
	{ 0x01, sizeof(struct SettingsTransportT), ProcessSettingsCommand },
	{ 0x02, sizeof(struct MessageTransportT), ProcessMessageCommand },
	{ 0x55, 0, ProcessTestCommand}
};

static int8_t TEMPLATE_Init(void)
{
	USBD_CDC_SetTxBuffer(&USBD_Device, bufferTx, 0);
	USBD_CDC_SetRxBuffer(&USBD_Device, bufferRx);
	QueueBuffer_Init(&packetQueue, packetQueueBuffer, PACKET_BUFFER_SIZE);
	return USBD_OK;
}

static int8_t TEMPLATE_DeInit(void)
{ 
	return USBD_OK;
}

static int8_t TEMPLATE_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
	switch (cmd)
	{
		case CDC_SEND_ENCAPSULATED_COMMAND:
			break;

		case CDC_GET_ENCAPSULATED_RESPONSE:
			break;

		case CDC_SET_COMM_FEATURE:
			break;

		case CDC_GET_COMM_FEATURE:
			break;

		case CDC_CLEAR_COMM_FEATURE:
			break;

		// Dummy store line coding from the host
		case CDC_SET_LINE_CODING:
			fakeBaud = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
			fakeFormat = pbuf[4];
			fakeParity = pbuf[5];
			fakeDatetype = pbuf[6];
			break;

		// Dummy readback whatever the host sent previously
		case CDC_GET_LINE_CODING:
			pbuf[0] = (uint8_t)(fakeBaud);
			pbuf[1] = (uint8_t)(fakeBaud >> 8);
			pbuf[2] = (uint8_t)(fakeBaud >> 16);
			pbuf[3] = (uint8_t)(fakeBaud >> 24);
			pbuf[4] = fakeFormat;
			pbuf[5] = fakeParity;
			pbuf[6] = fakeDatetype;  
			break;

		case CDC_SET_CONTROL_LINE_STATE:
			break;

		case CDC_SEND_BREAK:
			break;    
    
		default:
			break;
	}
	return USBD_OK;
}

static int8_t TEMPLATE_Receive(uint8_t* Buf, uint32_t *Len)
{
	// Append to the buffer
	QueueBuffer_AppendBuffer(&packetQueue, Buf, *Len);

	// Try to parse commands
	UsbCommandParse();

	// We can now get the next packet
	USBD_CDC_ReceivePacket(&USBD_Device);

	return USBD_OK;
}

static const struct CommandT* GetCommand(const uint8_t command)
{
	uint32_t i = 0;

	for (; i < COMMAND_COUNT; i++)
	{
		if (commands[i].Ident == command)
		{
			return &commands[i];
		}
	}

	return NULL;
}

static void ProcessSettingsCommand(const uint8_t* pointer)
{
	struct SettingsTransportT* transport = (struct SettingsTransportT*)pointer;

	printf("%.6s\r\n", transport->Callsign);
	printf("%.7s\r\n", transport->Path);
	
	printf("here\r\n");
}

static void ProcessTestCommand(const uint8_t* pointer)
{
	printf("Test!\r\n");
}

static void ProcessMessageCommand(const uint8_t* pointer)
{
	struct MessageTransportT* transport = (struct MessageTransportT*)pointer;
	RadioPacket_t beaconPacket;
	ConfigT* config = FlashConfigGetPtr();
	QueueHandle_t* txQueue = Radio_GetTxQueue();
	uint8_t payload[84];
	uint8_t payload_pointer = 0;
	uint8_t i;

	// If we didn't get any queues, something is really wrong
	if (txQueue == NULL)
	{
		return;
	}

	if (transport->MessageLength > 67)
	{
		return;
	}

	if (transport->MessageNumber > 99999)
	{
		return;
	}
	
	// Fill out radio packet
	memcpy(beaconPacket.frame.source, config->Aprs.Callsign, 6);
	beaconPacket.frame.source_ssid = config->Aprs.Ssid;
	memcpy(beaconPacket.frame.destination, "APRS  ", 6);
	beaconPacket.frame.destination_ssid = 0;
	memcpy(beaconPacket.path, config->Aprs.Path, 7);
	beaconPacket.frame.path_size = 7;
	beaconPacket.frame.pre_flag_count = 25;
	beaconPacket.frame.post_flag_count = 25;
	
	// APRS 1.0.1 page 71
	payload[payload_pointer++] = ':';

	// Copy in destination address
	for (i = 0; i < 9; i++)
	{
		if (transport->DestinationCallsign[i] > 0)
		{
			payload[payload_pointer++] = transport->DestinationCallsign[i];
		}
		else
		{
			payload[payload_pointer++] = ' ';
		}
	}

	payload[payload_pointer++] = ':';

	memcpy(&payload[payload_pointer], transport->Message, transport->MessageLength);
	payload_pointer += transport->MessageLength;

	payload[payload_pointer++] = '{';

	// Format message number
	snprintf((char*)&payload[payload_pointer], 5, "%lu", transport->MessageNumber);
	payload_pointer += log10(transport->MessageNumber) + 1;

	// Copy in payload
	memcpy(beaconPacket.payload, payload, payload_pointer);
	beaconPacket.frame.payload_size = payload_pointer;

	// Enqueue the packet in the transmit buffer
	xQueueSendToBackFromISR(*txQueue, &beaconPacket, 0);
}

static uint8_t UsbCommandParse(void)
{
	uint8_t workingByte;

	// While we have unprocessed data
	while (parserIndex < QueueBuffer_Count(&packetQueue))
	{
		// Get the working byte
		workingByte = QueueBuffer_Peek(&packetQueue, parserIndex);

		// Packet parse state machine
		switch(parserState)
		{
			// Look for a header
			case PARSER_STATE_HEADER :
				// Check for header match
				if (workingByte == '$')
				{
					parserState = PARSER_STATE_COMMAND;
				}

				// Consume this byte no matter what
				QueueBuffer_Dequeue(&packetQueue, 1);
				parserIndex = 0;
				break;

			// Get the command, this will tell us length info
			case PARSER_STATE_COMMAND :
				// Try to fetch the command
				workingCommand = (struct CommandT*)GetCommand(workingByte);
				parserIndex++;

				// Check if we actually got a valid command
				if (workingCommand != NULL)
				{
					// Check if this command is payloadless
					if (workingCommand->Length == 0)
					{
						parserState = PARSER_STATE_CRC1;
					}
					else
					{
						parserState = PARSER_STATE_PAYLOAD;
					}
				}
				else
				{
					parserState = PARSER_STATE_HEADER;
				}
				break;

			// Advance through the payload
			case PARSER_STATE_PAYLOAD :
				xBuffer[parserIndex - 1] = workingByte;
				parserIndex++;

				// Check if there is enough data to advance to CRC
				if (parserIndex > workingCommand->Length)
				{
					parserState = PARSER_STATE_CRC1;
				}
				break;

			// Collect CRC MSB
			case PARSER_STATE_CRC1 :
				// Collect MSB
				crcPacket = (workingByte << 8);
				parserIndex++;
				parserState = PARSER_STATE_CRC0;
				break;

			// Collect CRC LSB
			case PARSER_STATE_CRC0:
				// Collect LSB
				crcPacket |= workingByte;
				parserIndex++;

				// Verify CRC
				//if(CrcCcitt(xBuffer, workingLength) == crcPacket)
				if (crcPacket == 0x5555)
				{
					// Process the packet
					workingCommand->Handler(xBuffer);

					// Dequeue the entire packet
					QueueBuffer_Dequeue(&packetQueue, parserIndex);
					parserIndex = 0;
				}

				// No matter what, reset back to looking for a header
				parserState = PARSER_STATE_HEADER;
				break;
		}
	}

	return 1;
}