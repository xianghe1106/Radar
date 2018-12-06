#ifndef _QUEUE_H
#define _QUEUE_H

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
	INT32U Size;
	INT32U Entries;
	
	INT8U  *pStart;
	INT8U  *pEnd;
	
	INT8U  *pIn;
	INT8U  *pOut;
//	INT8U  *pBuf;
}QUEUE8_Type;

typedef enum
{
	QUEUE_NO_ERROR			= 0,
	QUEUE_POINT_ERROR,
	
	QUEUE_FULL,
	QUEUE_EMPTY,

}QUEUE_ERROR;

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




/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

INT8U QUEUE8_Create(QUEUE8_Type *pQ8, INT8U *pBuf, INT32U size);

INT8U QUEUE8_Push(QUEUE8_Type *pQ8, INT8U data);
INT8U QUEUE8_PushNData(QUEUE8_Type *pQ8, INT8U *pData, INT32U num);

INT8U QUEUE8_Pop(QUEUE8_Type *pQ8, INT8U *pData);
INT8U QUEUE8_PopNData(QUEUE8_Type *pQ8, INT8U *pData, INT32U num);

INT8U QUEUE8_GetSize(QUEUE8_Type *pQ8, INT32U *pSize);
INT8U QUEUE8_GetEntries(QUEUE8_Type *pQ8, INT32U *pEntries);

INT8U QUEUE8_Clear(QUEUE8_Type *pQ8);

/*
*********************************************************************************************************
*                                               END
*********************************************************************************************************
*/

#endif
