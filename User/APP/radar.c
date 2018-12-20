/*
 * radar.c
 *
 *  Created on: Dec 6, 2018
 *      Author: xianghe
 */


/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include "radar.h"
#include "driver.h"
#include <math.h>
#include "HostCommUART.h"
#include "xmc_uart.h"

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

QUEUE16_Type crest_I_queue, *p_crest_I_queue = &crest_I_queue;
QUEUE16_Type trough_I_queue, *p_trough_I_queue = &trough_I_queue;

INT16U crest_I_buffer[SAMPLING_SIZE];
INT16U trough_I_buffer[SAMPLING_SIZE];

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void RADAR_TestTime(void);
void radarsense2gol_result( uint32_t *fft_magnitude_array,
		uint16_t size_of_array_mag,
		int16_t *adc_aqc_array_I,
		int16_t *adc_aqc_array_Q,
		uint16_t size_of_array_acq,
		XMC_RADARSENSE2GOL_MOTION_t motion,
		uint32_t max_frq_mag,
		uint32_t max_frq_index);

void Radar_SearchMaxMinSamplingData(INT16U *input, INT16U size, INT16U *output);


void RADAR_Test(void)
{
	DIGITAL_GPIO_ToggleOutput(&LED_RED);
//	DIGITAL_GPIO_ToggleOutput(&LED_ORANGE);
//	DIGITAL_GPIO_ToggleOutput(&LED_BLUE);
}


void RADAR_Init(void)
{
	radarsense2gol_init(
			radarsense2gol_timing,
			radarsense2gol_algorithm,
			radarsense2gol_powerdown,
			&TIMER_0
	);

	// register call backs
	radarsense2gol_regcb_result ( radarsense2gol_result );
	radarsense2gol_exitmain();

	QUEUE16_Create(p_crest_I_queue, crest_I_buffer, SAMPLING_SIZE);
	QUEUE16_Create(p_trough_I_queue, trough_I_buffer, SAMPLING_SIZE);
}

void RADAR_Process(void)
{
	static bool running = false;

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
}



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
	INT16U buffer[2];

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

	Radar_SearchMaxMinSamplingData(g_sampling_data_I, BUFF_SIZE, buffer);
	QUEUE16_PushEx(p_crest_I_queue, buffer[0]);		//push into crest buffer
	QUEUE16_PushEx(p_trough_I_queue, buffer[1]);	//push into trough buffer

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

void Radar_SearchMaxMinSamplingData(INT16U *input, INT16U size, INT16U *output)
{
	INT16U i;

	output[0] = input[0];
	output[1] = input[0];

	for(i = 0; i < size; i++)
	{
		if(input[i] > output[0])
		{
			output[0] = input[i];	//serch max data
		}

		if(input[i] < output[1])
		{
			output[1] = input[i];	//search min data
		}
	}
}


void Radar_PrintCrestData(void)
{
	INT16U i;

#if 0
	XMC_UART_CH_Transmit(XMC_UART0_CH0, 0x7E);
	for(i = 0; i < SAMPLING_SIZE; i++)
	{
		XMC_UART_CH_Transmit(XMC_UART0_CH0, WORD_HIGH(crest_I_buffer[i]));
		XMC_UART_CH_Transmit(XMC_UART0_CH0, WORD_LOW(crest_I_buffer[i]));
	}
	XMC_UART_CH_Transmit(XMC_UART0_CH0, 0x7E);
#endif

#if 0
	XMC_UART_CH_Transmit(XMC_UART0_CH0, 0x7D);
	for(i = 0; i < SAMPLING_SIZE; i++)
	{
		XMC_UART_CH_Transmit(XMC_UART0_CH0, WORD_HIGH(trough_I_buffer[i]));
		XMC_UART_CH_Transmit(XMC_UART0_CH0, WORD_LOW(trough_I_buffer[i]));
	}
	XMC_UART_CH_Transmit(XMC_UART0_CH0, 0x7D);
#endif
}


/*
 * API
 */

void RADAR_GetDistance(INT8U *output)
{

}

void RADAR_GetSpeed(INT8U *output)
{
	INT16U speed = 0;

//	Float_type float_type;

	speed = (10 * g_doppler_velocity);


//	speed = speed;

	output[0] = WORD_HIGH(speed);
	output[1] = WORD_LOW(speed);


//	XMC_UART_CH_Transmit(XMC_UART0_CH0, speed & 0xFF);

}

void RADAR_GetSignal(INT8U *output)
{

}

void RADAR_GetAmplitude(INT8U *output)
{

}

void RADAR_GetMotion(INT8U *output)
{
//	return g_motion;
}



