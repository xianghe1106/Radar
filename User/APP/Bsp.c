/*
*********************************************************************************************************
*                                                BSP
*
* File    : BSP.c
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

#include "XMC1300.h"
#include "cpu.h"

#include "SCH_Core.h"

#include "driver.h"
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

void BSP_SysTickInit(void)
{
///	SysTick_Config(SystemCoreClock / TICKS_PER_SECOND);

	INT32U cnts;

	cnts = SystemCoreClock / SCH_CFG_TICK_RATE_HZ;

	if(SysTick_Config(cnts))
	{
		while(1);	/* Capture error */
	}
}


void BSP_HardwareInit(void)
{
	/*
	Note, the Delay_Init() function must be called the sooner the better,
	other functions may call DelayUs() or DelayMs() function.
	*/
//	Delay_Init();
	
//	Driver_led_init();

	Multi_IO_Init();
	Driver_uart_init();

	RADAR_Init();

}
/*------------------------------------------------------------------*-

  BSP_IntDis()

  All cpu interrupt disabled
  Avoid external or internal interrupt interrupted MCU initialization
  when MCU startup. 

-*------------------------------------------------------------------*/
void BSP_IntDis(void)
{
    CPU_IntDis();
}
/*------------------------------------------------------------------*-

  BSP_IntEn()

  When completed the initial work and then open the global interrupt

-*------------------------------------------------------------------*/
void BSP_IntEn(void)
{
    CPU_IntEn();
}

void BSP_WatchdogInit(void)
{

}

void BSP_FeedDog(void)
{

}


/*------------------------------------------------------------------*-

  Bsp_ExceptionHandle()
  
  Mainly deal with RCC_FLAG_PORRST or RCC_FLAG_SFTRST

-*------------------------------------------------------------------*/
void Bsp_ExceptionHandle(void)
{

}

/*------------------------------------------------------------------*-

  Bsp_SoftReset()
  
  Restart firmware

-*------------------------------------------------------------------*/
void Bsp_SoftReset(void)
{

}
/*------------------------------------------------------------------*-

  Bsp_SystemReset()
  
  SystemReset

-*------------------------------------------------------------------*/
void Bsp_SystemReset(void)
{
	__NVIC_SystemReset();
}
















