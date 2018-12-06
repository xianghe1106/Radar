/*
*********************************************************************************************************
*                                                queue
*
* File    : queue.c
* By      : XH
* Version : V1.0
*
* Comments:
* ---------------
*     
*********************************************************************************************************
*/ 



/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include "queue.h"

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



/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/



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




//QUEUE8_Type *QUEUE8_Create(INT8U *pBuf, INT32U size)
INT8U QUEUE8_Create(QUEUE8_Type *pQ8, INT8U *pBuf, INT32U size)
{
	if((pQ8 == (void *)0)
	|| (pBuf == (void *)0))
	{
		return QUEUE_POINT_ERROR;
	}

	pQ8->Size    = size;
	pQ8->Entries	= 0;
	
	pQ8->pIn		= pBuf;
	pQ8->pOut	= pBuf;
	
	pQ8->pStart  = pBuf;
	pQ8->pEnd    = &pBuf[size];
 
	return QUEUE_NO_ERROR;
}

INT8U QUEUE8_Push(QUEUE8_Type *pQ8, INT8U data)
{
	if(pQ8 == (void *)0)
	{
		return QUEUE_POINT_ERROR;
	}
	
	if(pQ8->Entries >= pQ8->Size)
	{
		return QUEUE_FULL;
	}
	
	*pQ8->pIn = data;
	
	pQ8->Entries++;	
	
	pQ8->pIn++;
	if(pQ8->pIn == pQ8->pEnd)
	{
		pQ8->pIn = pQ8->pStart;
	}
	
	return QUEUE_NO_ERROR;
}


INT8U QUEUE8_PushNData(QUEUE8_Type *pQ8, INT8U *pData, INT32U num)
{
	INT32U count = num;
	
	if((pQ8 == (void *)0)
	|| (pData == (void *)0))
	{
		return QUEUE_POINT_ERROR;
	}
	
	while(count--)
	{
		if(pQ8->Entries >= pQ8->Size)
		{
			return QUEUE_FULL;
		}
		
		*pQ8->pIn = *pData;
		
		pData++;
		pQ8->Entries++;	
		
		pQ8->pIn++;
		if(pQ8->pIn == pQ8->pEnd)
		{
			pQ8->pIn = pQ8->pStart;
		}
	}
	
	return QUEUE_NO_ERROR;
}

INT8U QUEUE8_Pop(QUEUE8_Type *pQ8, INT8U *pData)
{
	if((pQ8 == (void *)0)
	|| (pData == (void *)0))
	{
		return QUEUE_POINT_ERROR;
	}
	
	if(pQ8->Entries == 0)
	{
		return QUEUE_EMPTY;
	}
	
	*pData = *pQ8->pOut;
	
	pQ8->Entries--;		
	pQ8->pOut++;
	if(pQ8->pOut == pQ8->pEnd)
	{
		pQ8->pOut = pQ8->pStart;
	}


	return QUEUE_NO_ERROR;
}

INT8U QUEUE8_PopNData(QUEUE8_Type *pQ8, INT8U *pData, INT32U num)
{
	INT32U count = num;
	
	if((pQ8 == (void *)0)
	|| (pData == (void *)0))
	{
		return QUEUE_POINT_ERROR;
	}
	
	while(count--)
	{
		if(pQ8->Entries == 0)
		{
			return QUEUE_EMPTY;
		}
		
		*pData = *pQ8->pOut;
		
		pData++;
		pQ8->Entries--;		
		pQ8->pOut++;
		if(pQ8->pOut == pQ8->pEnd)
		{
			pQ8->pOut = pQ8->pStart;
		}
	}

	return QUEUE_NO_ERROR;
}

INT8U QUEUE8_GetSize(QUEUE8_Type *pQ8, INT32U *pSize)
{
	if((pQ8 == (void *)0)
	|| (pSize == (void *)0))
	{
		return QUEUE_POINT_ERROR;
	}

	*pSize = pQ8->Size;
	
	return QUEUE_NO_ERROR;
}

/*
*********************************************************************************************************
*				INT8U QUEUE8_GetEntries(QUEUE8_Type *pQ8, INT32U *pEntries)
*
* Description: This function is used to get the message entries in the created queue.
*
* Arguments  : pQ8    is the queue point which has been created.
*
*
* Returns    : Error code.
*
* Note(s)     : (1) .
*
*               (2) .
*********************************************************************************************************
*/
INT8U QUEUE8_GetEntries(QUEUE8_Type *pQ8, INT32U *pEntries)
{
	if((pQ8 == (void *)0)
	|| (pEntries == (void *)0))
	{
		return QUEUE_POINT_ERROR;
	}
	
	*pEntries = pQ8->Entries;
	
	return QUEUE_NO_ERROR;
}

/*
*********************************************************************************************************
*				INT8U QUEUE8_Clear(QUEUE8_Type *pQ8)
*
* Description: This function is used to clear the queue which has been created.
*
* Arguments  : pQ8    is the queue point which has been created.
*
*
* Returns    : Error code.
*
* Note(s)     : (1) .
*
*               (2) .
*********************************************************************************************************
*/
INT8U QUEUE8_Clear(QUEUE8_Type *pQ8)
{
	if(pQ8 == (void *)0)
	{
		return QUEUE_POINT_ERROR;
	}
	
	pQ8->Entries = 0;
	
	pQ8->pIn	 = pQ8->pStart;
	pQ8->pOut	 = pQ8->pStart;
	
	return QUEUE_NO_ERROR;
}



