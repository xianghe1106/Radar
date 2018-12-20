/*
 * radar.h
 *
 *  Created on: Dec 6, 2018
 *      Author: xianghe
 */

#ifndef USER_APP_RADAR_H_
#define USER_APP_RADAR_H_

/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include "Dave.h"
#include "cpu.h"
#include "radarsense2gol_library.h"
#include "config.h"
#include "xmc_uart.h"

/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                             DATA TYPES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/

#define BUFF_SIZE (FFT_SIZE)		/**< I and Q raw samples buffer size */
#define FFT_BIN_SIZE ((float)SAMPLING_FREQ_HZ / FFT_SIZE) /**< size of each FFT bin. DO NOT CHANGE!!! */

extern uint16_t g_sampling_data_I[BUFF_SIZE];				 /**< raw data i channel */
extern uint16_t g_sampling_data_Q[BUFF_SIZE];

/*
*********************************************************************************************************
*                                               MACRO'S
*********************************************************************************************************
*/

#define SAMPLING_SIZE							100


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void RADAR_Test(void);

void RADAR_Init(void);

void RADAR_Process(void);

void Radar_PrintCrestData(void);


/*API*/
void RADAR_GetDistance(INT8U *output);
void RADAR_GetSpeed(INT8U *output);
void RADAR_GetSignal(INT8U *output);
void RADAR_GetAmplitude(INT8U *output);
void RADAR_GetMotion(INT8U *output);


#endif /* USER_APP_RADAR_H_ */
