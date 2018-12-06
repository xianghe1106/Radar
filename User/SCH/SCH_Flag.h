#ifndef _SCH_FLAG_H
#define _SCH_FLAG_H

/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include "cpu.h"

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

typedef struct 
{
   INT8U Type;  

   INT16U Period;       

//   INT8U Status;       
}SCH_Flag_Type;


/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                               MACRO'S
*********************************************************************************************************
*/


#define SCH_MAX_FLAG								50


typedef enum
{
	SCH_FLAG_10MS  = 1,
	SCH_FLAG_20MS  = 2,
	SCH_FLAG_30MS  = 3,
	SCH_FLAG_40MS  = 4,
	SCH_FLAG_50MS  = 5,
	SCH_FLAG_60MS  = 6,
	SCH_FLAG_70MS  = 7,
	SCH_FLAG_80MS  = 8,
	SCH_FLAG_90MS  = 9,
	SCH_FLAG_100MS = 10,
	SCH_FLAG_150MS = 15,
	SCH_FLAG_200MS = 20,
	SCH_FLAG_250MS = 25,
	SCH_FLAG_300MS = 30,
	SCH_FLAG_350MS = 35,
	SCH_FLAG_400MS = 40,
	SCH_FLAG_450MS = 45,
	SCH_FLAG_500MS = 50,

	SCH_FLAG_1000MS = 100,
	SCH_FLAG_1500MS = 150,
	SCH_FLAG_2000MS = 200,
	SCH_FLAG_2500MS = 250,
	SCH_FLAG_3000MS = 300,
	SCH_FLAG_3500MS = 350,
	SCH_FLAG_4000MS = 400,
	SCH_FLAG_4500MS = 450,
	SCH_FLAG_5000MS = 500,

}SCH_FLAG_TIME;

typedef enum{SCH_FLAG_FULL = SCH_MAX_FLAG,SCH_FLAG_REGISTERED = (SCH_MAX_FLAG + 1)}SCH_FLAG_ERROR_STATUS;




/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void SCH_Flag_Init(void);

INT8U SCH_Add_Flag(const INT8U TYPE,const INT16U PERIOD);

void SCH_Flag_Update(void);

INT16U SCH_Get_Flag(const INT8U Type);


#endif


