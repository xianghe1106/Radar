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
	INT8U frame_flag;
	INT8U buffer[50];
}PACKET_DATA_Type;

typedef struct
{
	INT8U store_index;
	INT8U process_index;
	INT8U buffer[PACKET_MAX_ENTRIES][PACKET_MAX_LEN];
}PACKET_BUFFER_Type;

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
INT8U  tx_bufer[50];

//INT8U  packet_buffer[50];

QUEUE8_Type rx_queue, *p_rx_queue = &rx_queue;

INT32U prtocol_time = 0;

PACKET_DATA_Type preprocess_buffer/*, process_buffer*/;
PACKET_BUFFER_Type packet_buffer;

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/
#if 0
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
//	{0xC2 	, GetLEDType							,1 },
//	{0xC3	, GetSNMPReadCommunity					,0 },


	/*---------------------------------------------------
	--
	-- Internal calibration command: 0xE0 to 0xFE
	--
	----------------------------------------------------*/
//	{0xC2 	, GetLEDType							,1 },


	{0xFFFF , NULL									,0 }
};
#endif

void Protocol_init(void)
{
	QUEUE8_Create(p_rx_queue, rx_buffer, sizeof(rx_buffer));

	MEM_Clr(&preprocess_buffer.count, 	sizeof(preprocess_buffer));
//	MEM_Clr(&process_buffer.count, 		sizeof(process_buffer));
	MEM_Clr(&packet_buffer.store_index, sizeof(packet_buffer));
	//packet_buffer
}


void USIC0_1_IRQHandler(void)
{
	INT8U data;

	data = XMC_UART_CH_GetReceivedData(XMC_UART0_CH0);  // receive data
	QUEUE8_Push(p_rx_queue, data);

//	XMC_UART_CH_Transmit(XMC_UART0_CH0, received_data);
//	DIGITAL_IO_ToggleOutput(&LED_BLUE);
}

void send_uart_data(INT8U *p_buffer, INT8U count)
{
	INT8U i;

	for(i = 0; i < count; i++)
	{
		XMC_UART_CH_Transmit(XMC_UART0_CH0, p_buffer[i]);
	}
}

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

void scan_packet(void)
{
	INT8U i, start_index = 0, stop_index = 0, frame_flag = 0;
	INT8U size, data=0, index = 0;
	INT8S offset=0;

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

	if(frame_flag == 2)//Found a packet.
	{
//		MEM_Clr(&packet_buffer.buffer[packet_buffer.store_index][0], sizeof(packet_buffer.buffer[packet_buffer.store_index]));

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

void Protocol_process(void)
{
	Protocol_preprocessing();

	if(packet_buffer.process_index == packet_buffer.store_index)
	{
		return;
	}

	send_uart_data(&packet_buffer.buffer[packet_buffer.process_index][0], 9);

	packet_buffer.process_index++;
	packet_buffer.process_index = packet_buffer.process_index % PACKET_MAX_ENTRIES;
}

/*	send_uart_data(&packet_buffer.buffer[packet_buffer.process_index][0], sizeof(packet_buffer.bufferpacket_buffer.[process_index]));

	packet_buffer.process_index++;
	packet_buffer.process_index = packet_buffer.process_index % PACKET_MAX_ENTRIES;
	return;*/


/*	XMC_UART_CH_Transmit(XMC_UART0_CH0, 0xAA);
	send_uart_data(preprocess_buffer.buffer, preprocess_buffer.count);
	XMC_UART_CH_Transmit(XMC_UART0_CH0, 0xAA);


	process_buffer.frame_flag = 0;
	return;*/

/*
#if 0

	if(QUEUE8_GetEntries(p_rx_queue, &queue_entries) != QUEUE_NO_ERROR)
	{
		return;
	}

	if(queue_entries < 4)
	{
		return;
	}

	MEM_Clr(packet_buffer, sizeof(packet_buffer));
	Prtocol_time_clear();

	while((Protocol_get_time() < 2) //at least 10ms
			&& (queue_entries > 0))
	{

		QUEUE8_Pop(p_rx_queue, &queue_data);
		if(queue_data == start_byte)
		{
			packet_flag = flag_start;
		}

		switch(packet_flag)
		{
			case flag_start:

				if(queue_data == start_byte)
				{
					packet_buffer[0] = queue_data;

					packet_flag = flag_address;
				}
				else
				{
					return;
				}

				break;

			case flag_address:
				if(queue_data == 0x7D)
				{
					QUEUE8_Pop(p_rx_queue, &queue_next_data);
					if(queue_next_data == 0x5E)
					{
						packet_buffer[1] = start_byte;
					}
					else if(queue_next_data == 0x5D)
					{
						packet_buffer[1] = 0x7D;
					}
					else
					{
						packet_buffer[1] = 0x7D;
						packet_buffer[2] = queue_next_data;
					}
				}
				else
				{
					packet_buffer[1] = queue_data;
				}

				packet_flag = flag_length;

				break;

			case flag_length:
				if(packet_buffer[2] == 0x00)	//first time write data to packet_buffer[2]
				{

				}
				else
				{	//case flag_address have write a data to packet_buffer[2]
					if()
					{

					}
				}

				break;

			case flag_data:
				break;

			case flag_crc:
				break;

			default:
				break;

		}

		QUEUE8_GetEntries(p_rx_queue, &queue_entries);
	}



	QUEUE8_PopNData(p_rx_queue, tx_data, queue_entries);

#endif

//	send_uart_data(tx_data, queue_entries);
//}
*/


