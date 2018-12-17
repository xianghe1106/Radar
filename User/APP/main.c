/**
 * \file main.c
 * \brief This file implements the top-level functionality for the motion detection on Sense2GoL radar board.
 * UART feature is also used in this implementation to dump IQ raw data samples to the HOST.
 * UART Configurations: Full-duplex, Direct mode, 9600 baud rate, 8 data-bits, 1 stop-bit, no parity
 * Data format of transmission: Completely transmits I_adc samples of buffer length BUFF_SIZE,
 * followed by Q_adc samples of same buffer length of BUFF_SIZE so in total 2 * BUFF_SIZE samples are transmitted
 *
 * \author Assad Ali
 *
 * \cond
 * ********************************************************************************************
 * Copyright (c) 2017, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * To improve the quality of the software, users are encouraged to share modifications, enhancements or bug fixes
 * with Infineon Technologies AG (thomas.finke@infineon.com).
 * ********************************************************************************************
 * \endcond
 *
 */

/****************************************************************
 * HEADER FILES
 ***************************************************************/
#include "Dave.h"
#include "radarsense2gol_library.h"
#include "HostCommUART.h"
#include "config.h"

#include "SCH_Core.h"
#include "radar.h"
#include "Protocol.h"
#include "Bsp.h"

#include "driver.h"

/**********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/
#define BUFF_SIZE (FFT_SIZE)		/**< I and Q raw samples buffer size */
#define FFT_BIN_SIZE ((float)SAMPLING_FREQ_HZ / FFT_SIZE) /**< size of each FFT bin. DO NOT CHANGE!!! */

/**********************************************************************************************************************
 * Micirum uC Probe variables
 **********************************************************************************************************************/

uint16_t g_sampling_data_I[BUFF_SIZE] = {0};				 /**< raw data i channel */
uint16_t g_sampling_data_Q[BUFF_SIZE] = {0};				 /**< raw data q channel */
uint32_t g_fft_data[FFT_SIZE/2] = {0};						 /**< fft data of i channel */
XMC_RADARSENSE2GOL_MOTION_t g_motion = XMC_NO_MOTION_DETECT; /**< motion indicator */
float g_doppler_frequency = 0.0;							 /**< doppler frequency */
float g_doppler_velocity = 0.0;								 /**< doppler speed */
float g_min_velocity = MOTION_MIN_VELOCITY; 				 /**< min velocity to detect motion */
float g_max_velocity = MOTION_MAX_VELOCITY; 				 /**< max velocity to detect motion */
bool g_start = true;										 /**< control for execution of doppler algorithm */
bool g_uart_start = UART_RAW_DATA;							 /**< control for execution of UART feature to transmit raw IQ from ADC */

void RADAR_TestTime(void);
/************************************************************************************************************/

/*!
 * \brief Setup the timings related parameters
 */

XMC_RADARSENSE2GOL_TIMING_t radarsense2gol_timing =
{
		.t_sample_us = (1.0 / SAMPLING_FREQ_HZ) * 1000.0 * 1000.0, /**< sample time in us = (1/sample_frequency) * 1000 * 1000 */
		.t_cycle_ms = 300,           /**< 300 ms */
		.N_exponent_samples = log2(BUFF_SIZE)
};


/*!
 * \brief Setup the detection triggers related parameters
 */
XMC_RADARSENSE2GOL_ALG_t radarsense2gol_algorithm =
{
		.hold_on_cycles = 1,      /**< hold-on cycles to trigger detection */
		.trigger_det_level = DETECTION_THRESHOLD,  /**< trigger detection level */
		.rootcalc_enable = XMC_RADARSENSE2GOL_DISABLED /**< root calculation for magnitude disabled */
};

/*!
 * \brief Setup the power management related parameters
 */
XMC_RADARSENSE2GOL_POWERDOWN_t radarsense2gol_powerdown =
{
		.sleep_deepsleep_enable   = XMC_RADARSENSE2GOL_ENABLED, /**< sleep / deepsleep enabled */
		.mainexec_enable          = XMC_RADARSENSE2GOL_ENABLED, /**< main exec enabled */
		.vadc_clock_gating_enable = XMC_RADARSENSE2GOL_ENABLED  /**< vadc clock gating enabled */
};

/************************************************************************************************************/

/*! max_frw_index gives the fft bin in which the maximum magnitude of the fft was found each fft bin has a size
 * and therefore corresponds to a frequency range the size of the fft bin is calculated by dividing the sampling frequency
 * by the number of fft points the doppler frequency is calculated by multiplying the fft bin size with the number
 * of the fft bin where the maximum was found.
 */
static float calcDopplerFrequency(const uint32_t max_frq_index)
{
	return (float)(max_frq_index * FFT_BIN_SIZE);
}

/************************************************************************************************************/

/*! doppler velocity is calculated based on the doppler frequency
 * for 24GHz doppler radar systems a velocity of 1km/h will correspond to a doppler frequency of 44.4Hz
 * doppler velocity can therefore be calculated by dividing the doppler frequency by 44.4Hz
 */

static float calcDopplerSpeed(const float doppler_freq)
{
	return doppler_freq / 44.4f;
}

/************************************************************************************************************/

/*!
 * \brief Callback executed after new data is available from algorithm.
 *
 * \param[in] *fft_magnitude_array  Array pointer to the FFT spectrum
 * \param[in] size_of_array_mag  Length of array for the FFT spectrum
 * \param[in] *adc_aqc_array_I   Array pointer for the raw ADC I data samples
 * \param[in] *adc_aqc_array_Q   Array pointer for the raw ADC Q data samples
 * \param[in] size_of_array_acq  Length of array for the raw ADC I&Q data samples
 * \param[out] motion  Approaching, departing or no motion information
 * \param[out] max_frq_mag  Maximum doppler frequency computed
 * \param[out] max_frq_index  Maximum frequency bin index above threshold
 *
 */
void radarsense2gol_result( uint32_t *fft_magnitude_array,
		uint16_t size_of_array_mag,
		int16_t *adc_aqc_array_I,
		int16_t *adc_aqc_array_Q,
		uint16_t size_of_array_acq,
		XMC_RADARSENSE2GOL_MOTION_t motion,
		uint32_t max_frq_mag,
		uint32_t max_frq_index)
{
	// copy raw data and fft data and motion indicator to global variables used in micrium GUI
	memcpy(g_sampling_data_I, adc_aqc_array_I, size_of_array_acq * sizeof(uint16_t));
	memcpy(g_sampling_data_Q, adc_aqc_array_Q, size_of_array_acq * sizeof(uint16_t));
	memcpy(g_fft_data, &fft_magnitude_array[1], (size_of_array_mag - 1) * sizeof(uint32_t));

	// To remove the spike from last bins in fft spectrum, force last two bins to 0
	g_fft_data[size_of_array_mag-1] = 0;
	g_fft_data[size_of_array_mag-2] = 0;

	fft_magnitude_array[size_of_array_mag-1] = 0;
	fft_magnitude_array[size_of_array_mag-2] = 0;

	g_motion = motion;

	// calc doppler frequency and velocity
	g_doppler_frequency = calcDopplerFrequency(max_frq_index);
	g_doppler_velocity = calcDopplerSpeed(g_doppler_frequency);

	if (g_uart_start)
		dumpRawIQ_uint16(g_sampling_data_I, g_sampling_data_Q, (uint16_t)BUFF_SIZE);

	// check results
	if (motion != XMC_NO_MOTION_DETECT &&			// motion detected
	    g_doppler_velocity > g_min_velocity &&		// doppler velocity is greater than min velocity
		g_doppler_velocity < g_max_velocity)		// doppker velocity is less than max veloctiy
	{
		if (motion == XMC_MOTION_DETECT_APPROACHING)	// target is approaching radar
		{
			// turn on red LED, turn off orange and blue LEDs
//			DIGITAL_IO_SetOutputLow(&LED_RED);
//			DIGITAL_IO_SetOutputHigh(&LED_ORANGE);
			DIGITAL_GPIO_SetOutputLow(&LED_BLUE);
		}
		else // motion == XMC_MOTION_DETECT_DEPARTING => target is moving away from radar
		{
			// turn on orange LED, turn off red and blue LEDs
			DIGITAL_GPIO_SetOutputLow(&LED_ORANGE);
//			DIGITAL_IO_SetOutputHigh(&LED_RED);
//			DIGITAL_IO_SetOutputHigh(&LED_BLUE);
		}
	}
	else // no motion detected
	{
		// turn on blue LED, turn off red and blue LEDs
		DIGITAL_GPIO_SetOutputHigh(&LED_BLUE);
		DIGITAL_GPIO_SetOutputHigh(&LED_ORANGE);
//		DIGITAL_IO_SetOutputHigh(&LED_RED);

		// set velocity and frequency to 0 in case of no motion
		g_doppler_frequency = 0.0;
		g_doppler_velocity = 0.0;
		g_motion = XMC_NO_MOTION_DETECT;
	}
}

/************************************************************************************************************/

/*!
 * \brief Top-level function for the motion detection using radar Sense2GoL board
 */

void TASK_uart(void);

int main(void)
{
	bool running = false;
//	SCH_RTC_Type run_time_a, run_time_b;
//	INT16U delta_time;
//	INT8U tx_buffer[10];

	DAVE_Init();  //Initialization of DAVE APPs


	radarsense2gol_init(
			radarsense2gol_timing,
			radarsense2gol_algorithm,
			radarsense2gol_powerdown,
			&TIMER_0
	);

	// register call backs
	radarsense2gol_regcb_result ( radarsense2gol_result );
	radarsense2gol_exitmain();

	BSP_IntDis();

	SystemInit();

	BSP_HardwareInit();

	Protocol_init();

	SCH_Init();

	/* Add Task */
	SCH_Add_Task(RADAR_Test 			, 		1  , 		20   );

	SCH_Add_Task(Protocol_process 		, 		2  , 		10   );//RADAR_TestTime

	SCH_Add_Task(Protocol_heart_beat 	, 		10 , 		16   );//Protocol_heart_beat
//	SCH_Add_Task(Protocol_heart_beat 	, 		10 , 		100   );//Protocol_heart_beat

	SCH_Start();


	while (1)
	{
		SCH_Dispatch_Tasks();
		Protocol_preprocessing();
//		Protocol_process();

#if 1
//		run_time_a = SCH_Get_RTC();
		if (running == false)
		{
			if (g_start == true)
			{
				running = true;
				radarsense2gol_start();
			}
		}
		else
		{
			if (g_start == false)
			{
				running = false;
				radarsense2gol_stop();
			}

			radarsense2gol_set_detection_threshold(radarsense2gol_algorithm.trigger_det_level);

			/* place your application code for main execution here */
			/* e.g. communication on peripherals */
			radarsense2gol_exitmain(); /* only need to be called if mainexec_enable is enabled during init */

		}

//		run_time_b = SCH_Get_RTC();

//		delta_time = (run_time_b.minute * 60 * 1000 + run_time_b.second * 1000 + run_time_b.msec) - (run_time_a.minute * 60 * 1000 + run_time_a.second * 1000 + run_time_a.msec);

#endif
	}

}

void RADAR_TestTime(void)
{
	bool running = false;
//	SCH_RTC_Type run_time_a, run_time_b;
//	INT16U delta_time;
//	INT8U tx_buffer[10];

//	run_time_a = SCH_Get_RTC();
	if (running == false)
	{
		if (g_start == true)
		{
			running = true;
			radarsense2gol_start();
		}
	}
	else
	{
		if (g_start == false)
		{
			running = false;
			radarsense2gol_stop();
		}

		radarsense2gol_set_detection_threshold(radarsense2gol_algorithm.trigger_det_level);

		/* place your application code for main execution here */
		/* e.g. communication on peripherals */
		radarsense2gol_exitmain(); /* only need to be called if mainexec_enable is enabled during init */

	}

//	run_time_b = SCH_Get_RTC();

//	delta_time = (run_time_b.minute * 60 * 1000 + run_time_b.second * 1000 + run_time_b.msec) - (run_time_a.minute * 60 * 1000 + run_time_a.second * 1000 + run_time_a.msec);
}





