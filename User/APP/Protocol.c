/*
 * Protocol.c
 *
 *  Created on: Dec 11, 2018
 *      Author: xianghe
 */




/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include "Protocol.h"
#include "queue.h"
#include "mem.h"

#include "Dave.h"

#include "xmc_uart.h"
#include "radar.h"

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/

#define COMMAND_COUNT 						50
#define PACKET_MAX_LEN						52
#define PACKET_MAX_ENTRIES					5


const INT8U s_aucCrc8LoopupTbl[] =
{
       0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
       0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
       0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
       0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
       0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
       0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
       0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
       0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
       0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
       0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
       0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
       0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
       0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
       0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
       0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
       0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/

typedef enum
{
	flag_null 		= 0,
	flag_start 		= 1,
	flag_address 	= 2,
	flag_length 	= 3,
	flag_data 		= 4,
	flag_crc 		= 5
}PACKET_FLAG_Type;

typedef struct
{
	INT8U count;
	INT8U buffer[51];
}SCAN_PACKET_Type;

typedef struct
{
	INT8U store_index;
	INT8U process_index;
	INT8U buffer[PACKET_MAX_ENTRIES][PACKET_MAX_LEN];
}PACKET_BUFFER_Type;

ProtocolOffset_TypeDef protocol_offset;

/*
*********************************************************************************************************
*                                            LOCAL TABLES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

INT8U  rx_buffer[100];
INT8U  tx_buffer[300];


QUEUE8_Type rx_queue, *p_rx_queue = &rx_queue;

SCAN_PACKET_Type preprocess_buffer/*, process_buffer*/;
PACKET_BUFFER_Type packet_buffer;

INT8U  device_address = 0x30;

//INT16U distance, speed, signal, amplitude;
//INT8U  motion = 1;

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static INT8U zxUTILS_Crc8(INT8U *ucBuf, INT8U ucLen);

static void GetFirmwareVersion(ProtocolMsg_TypeDef ProtocolMsg);

/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/

const struct NEURON neuron[COMMAND_COUNT]=
{
	/*-------------------------------------------
	--
	-- EX_Protocol Set command : 0x01 to 0x3F
	--
	-------------------------------------------*/
//	{0x60 	, SetIPMask								,0 },
//	{0x61 	, SetIPGateway							,0 },


	/*------------------------------------------
	--
	-- EX_Protocol Get command : 0x40 to 0x6F
	--
	-------------------------------------------*/
//	{0x70 	, GetGainStep 							,1 },
//	{0x71 	, GetIPMask 							,4 },



	/*---------------------------------------------------
	--
	-- Internal set command : 0x80 to 0xAF
	--
	----------------------------------------------------*/
//	{0xA0 	, SetLEDType							,0 },


	/*---------------------------------------------------
	--
	-- Internal get command: 0xB0 to 0xDF
	--
	----------------------------------------------------*/
	{0xA0 	, GetFirmwareVersion					,2 },
//	{0xC3	, GetSNMPReadCommunity					,0 },


	/*---------------------------------------------------
	--
	-- Internal calibration command: 0xE0 to 0xFE
	--
	----------------------------------------------------*/
//	{0xC2 	, GetLEDType							,1 },


	{0xFFFF , NULL									,0 }
};


void Protocol_init(void)
{
	QUEUE8_Create(p_rx_queue, rx_buffer, sizeof(rx_buffer));
	MEM_Clr(&preprocess_buffer.count, 	sizeof(preprocess_buffer));
	MEM_Clr(&packet_buffer.store_index, sizeof(packet_buffer));
}


void USIC0_1_IRQHandler(void)
{
	INT8U data;

	data = XMC_UART_CH_GetReceivedData(XMC_UART0_CH0);  // receive data
	QUEUE8_Push(p_rx_queue, data);
}

void send_uart_data(INT8U *p_buffer, INT16U count)
{
	INT16U i;

	XMC_UART_CH_Transmit(XMC_UART0_CH0, 0x7E);
	for(i = 0; i < count; i++)
	{
		if((p_buffer[i] == 0x7E) || ((p_buffer[i] == 0x7D)))
		{
			XMC_UART_CH_Transmit(XMC_UART0_CH0, 0x7D);
			XMC_UART_CH_Transmit(XMC_UART0_CH0, (p_buffer[i] & 0xFF) ^ 0x20);
		}
		else
		{
			XMC_UART_CH_Transmit(XMC_UART0_CH0, p_buffer[i]);
		}
	}
	XMC_UART_CH_Transmit(XMC_UART0_CH0, 0x7E);
}

/*
void Protocol_time_update(void)
{
	prtocol_time++;
}

void Prtocol_time_clear(void)
{
	prtocol_time = 0;
}

INT32U Protocol_get_time(void)
{
	return prtocol_time;
}
*/

void scan_packet(void)
{
	INT8U i, start_index = 0, stop_index = 0, frame_flag = 0;
	INT8U size, index = 0;
	INT8S offset=0;

	if(preprocess_buffer.count == 0)
	{
		return;
	}

	for(i = 0; i < preprocess_buffer.count; i++)
	{
		if(preprocess_buffer.buffer[start_index++] == START_BYTE)
		{
			frame_flag++;

			if(preprocess_buffer.buffer[start_index] == START_BYTE)
			{
				start_index++;
			}
			break;
		}
	}

	for(i = 0; i < preprocess_buffer.count; i++)
	{
		if(preprocess_buffer.buffer[stop_index++] == START_BYTE)
		{
			if((stop_index - start_index) > 1)
			{
				frame_flag++;
				break;
			}
		}
	}

//	return;

	if(frame_flag == 2)//Found a packet.
	{
		MEM_Clr(&packet_buffer.buffer[packet_buffer.store_index][0], sizeof(packet_buffer.buffer[packet_buffer.store_index]));

		size = stop_index - start_index + 1;
		index = 0;
		for(i = 0; i < size; i++)
		{
			if(packet_buffer.buffer[packet_buffer.store_index][index] == 0x7D)
			{
				packet_buffer.buffer[packet_buffer.store_index][index] = preprocess_buffer.buffer[start_index - 1  + i] ^ 0x20;

				index++;
				offset--;
			}
			else
			{
				packet_buffer.buffer[packet_buffer.store_index][index] = preprocess_buffer.buffer[start_index - 1  + i];
				if(packet_buffer.buffer[packet_buffer.store_index][index] != 0x7D)
				{
					index++;
				}
			}
		}

		packet_buffer.store_index++;
		packet_buffer.store_index = packet_buffer.store_index % PACKET_MAX_ENTRIES;


		size = preprocess_buffer.count - stop_index;
		for(i = 0; i < (size + offset); i++)
		{
			preprocess_buffer.buffer[i] = preprocess_buffer.buffer[stop_index + i];
		}

		preprocess_buffer.count = size;
	}
}

void Protocol_preprocessing(void)
{
	INT32U queue_entries = 0;

	if(QUEUE8_GetEntries(p_rx_queue, &queue_entries) != QUEUE_NO_ERROR)
	{
		return;
	}

	if(queue_entries > 0)
	{
		QUEUE8_PopNData(p_rx_queue, &preprocess_buffer.buffer[preprocess_buffer.count], queue_entries);

		preprocess_buffer.count += queue_entries;
	}

	scan_packet();
}

INT16U Calculate_SUM_LRC(INT8U *Packet, INT8U length)
{
	INT16U CHECK_SUM = 0x0000;
	INT8U  i;

	for (i=0; i<length-1; i++)
	{
		CHECK_SUM = CHECK_SUM+Packet[i];
	}
	CHECK_SUM = CHECK_SUM & 0xFF;
	CHECK_SUM = ((CHECK_SUM ^ 0xFF)+1) & 0xFF;

    return CHECK_SUM;
}

void Protocol_heart_beat(void)
{
	INT8U  len;//tx_bufer
	INT8U  buffer[5];

	len = 9;

	tx_buffer[0] = device_address;	//device_adress
	tx_buffer[1] = 0;	//len
	tx_buffer[2] = len;	//len

	tx_buffer[3] = 0x01;	//command

	tx_buffer[4] = 0x00;	//error code

	RADAR_GetDistance(buffer);
	tx_buffer[5] = buffer[0];	//distance
	tx_buffer[6] = buffer[1];

	RADAR_GetSpeed(buffer);
	tx_buffer[7] = buffer[0];	//speed
	tx_buffer[8] = buffer[1];

	RADAR_GetMotion(buffer);
	tx_buffer[9] = buffer[0];

	RADAR_GetSignal(buffer);
	tx_buffer[10] = buffer[0];	//speed
	tx_buffer[11] = buffer[1];

	RADAR_GetAmplitude(buffer);
	tx_buffer[12] = buffer[0];	//speed
	tx_buffer[13] = buffer[1];

	tx_buffer[14] = zxUTILS_Crc8(&tx_buffer[0], len + 5) & 0xFF;	//check byte

	send_uart_data(tx_buffer, len + 6);
}

void Protocol_RaderSamplingData(void)
{
	INT16U len, size;
	INT8U  buffer[4];

	size = sizeof(g_sampling_data_I);
	len  = size + 1;


	XMC_UART_CH_Transmit(XMC_UART0_CH0, WORD_HIGH(g_sampling_data_I[0]));
	XMC_UART_CH_Transmit(XMC_UART0_CH0, WORD_LOW(g_sampling_data_I[0]));

	tx_buffer[0] = device_address;	//device_adress
	tx_buffer[1] = WORD_HIGH(len);	//len
	tx_buffer[2] = WORD_LOW(len);

	tx_buffer[3] = 0x02;	//command

	tx_buffer[4] = 0x00;	//error code

	MEM_Copy(&tx_buffer[5], g_sampling_data_I, sizeof(g_sampling_data_I));

	RADAR_GetMotion(buffer);
	tx_buffer[5 + size] = buffer[0];

	tx_buffer[len + 5] = zxUTILS_Crc8(&tx_buffer[0], len+5) & 0xFF;	//check byte

	send_uart_data(tx_buffer, len + 6);
}

void Protocol_process(void)
{
	INT8U  rxLen, errorCode=CMD_SUCCEED, dynamicLen=0, staticLen=0;
	INT16U cmdCode, index;
	INT16U crcValue;
//	INT8U  txLen = 0, paraCache[40];
//	INT8U  cacheSize;
	INT8U  packet_address;
	INT8U  *ptr_buffer;

	ProtocolMsg_TypeDef ProtocolMsg;

//	Protocol_preprocessing();

	if(packet_buffer.process_index == packet_buffer.store_index)
	{
		return;
	}

/*
	send_uart_data(&packet_buffer.buffer[packet_buffer.process_index][0], 6);

	packet_buffer.process_index++;
	packet_buffer.process_index = packet_buffer.process_index % PACKET_MAX_ENTRIES;
	return;
*/

	ptr_buffer = &packet_buffer.buffer[packet_buffer.process_index][0];

	rxLen = ptr_buffer[3];
	if((ptr_buffer[0] != START_BYTE)
		|| (ptr_buffer[rxLen + 1] != STOP_BYTE))
	{
//		XMC_UART_CH_Transmit(XMC_UART0_CH0, 0x01);

		packet_buffer.process_index++;
		packet_buffer.process_index = packet_buffer.process_index % PACKET_MAX_ENTRIES;
		return;
	}

	packet_address = ptr_buffer[1];
	if(packet_address != device_address)
	{
//		XMC_UART_CH_Transmit(XMC_UART0_CH0, 0x02);
		packet_buffer.process_index++;
		packet_buffer.process_index = packet_buffer.process_index % PACKET_MAX_ENTRIES;
		return;
	}

	crcValue = zxUTILS_Crc8(&ptr_buffer[1], rxLen - 1);
	if(crcValue != ptr_buffer[rxLen])
	{
		errorCode = CRC_ERROR;
	}

	cmdCode = ptr_buffer[4];
	if(errorCode == CMD_SUCCEED)
	{
		//process packet
		for(index = 0; index < COMMAND_COUNT; index++)
		{
			if(0xFFFF == neuron[index].Command)
			{
				staticLen = 0;
				errorCode = UNSUPPORT_CMD;
				break;
			}

			if(cmdCode == neuron[index].Command)
			{
				errorCode				= CMD_SUCCEED;	//reset
				dynamicLen				= 0x00; 		//reset

				staticLen 			    = neuron[index].DataLen;

				//input data
	//			ProtocolMsg.CommandType = cmdCode >> 15;
				ProtocolMsg.RxParaLen  	= ptr_buffer[3] - 5;	//number of parameters
				ProtocolMsg.Para        = &ptr_buffer[5];

				//output data
				ProtocolMsg.ErrorCode   = &errorCode;
	//			ProtocolMsg.Output      = &paraCache[0];   	//paraCache
				ProtocolMsg.Output      = &tx_buffer[5];   	//paraCache
				ProtocolMsg.DynamicLen  = &dynamicLen;
				(neuron[index].Cell)(ProtocolMsg);
				break;
			}

/*			if(index == (COMMAND_COUNT - 1))
			{
				staticLen = 0;
				errorCode = UNSUPPORT_CMD;
				break;
			}*/
		}
	}

	//preparing the response packet
	if(errorCode != CMD_SUCCEED)
	{
		staticLen = 0;
		dynamicLen = 0;
	}

	tx_buffer[0] = device_address;
	tx_buffer[1] = 0x00;
	tx_buffer[2] = staticLen + dynamicLen + 6;
	tx_buffer[3] = cmdCode;
	tx_buffer[4] = errorCode;

	crcValue = zxUTILS_Crc8(&tx_buffer[0], tx_buffer[2] - 1);
//	XMC_UART_CH_Transmit(XMC_UART0_CH0, crcValue);
	tx_buffer[tx_buffer[2] - 1] = crcValue;

	send_uart_data(tx_buffer, tx_buffer[2]);

	packet_buffer.process_index++;
	packet_buffer.process_index = packet_buffer.process_index % PACKET_MAX_ENTRIES;
}


static INT8U zxUTILS_Crc8(INT8U *ucBuf, INT8U ucLen)
{
	INT8U  crc = 0x00;

    while (ucLen--)
    {
        crc = s_aucCrc8LoopupTbl[crc ^ *ucBuf++];
    }
    return crc ^ 0x55;
}

static void GetFirmwareVersion(ProtocolMsg_TypeDef ProtocolMsg)
{
	ProtocolMsg.Output[0] = MAJOR_VERSION;
	ProtocolMsg.Output[1] = MINOR_VERSION;
}

