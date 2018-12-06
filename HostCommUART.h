/**
 *
 * \file HostCommUART.h
 * \brief This file defines the driver API for UART Communication for Sense2GoL radar board.
 * \author Assad Ali
 *
 * \cond
 ***********************************************************************************************************************
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
 ************************************************************************************************************************
 * \endcond
 *
 */

#ifndef HOSTCOMMUART_H_
#define HOSTCOMMUART_H_

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/

#include <DAVE.h>                 // Declarations from DAVE Code Generation (includes SFR declaration)
#include <stdio.h>
#include <math.h>
#include <inttypes.h>

/**********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/

#define COL_LENGTH	(16U) 		/**< number of columns to be displayed per row in the Host terminal */

/**********************************************************************************************************************
 * FUNCTIONS
 **********************************************************************************************************************/

/*!
 * \brief Transmits a complete string (IQ data samples are converted to character type (1-byte) for UART transmission).
 *
 * A string passed as an argument is actually type-casted character array of the IQ raw data samples.
 * Length of the string is determined by the position of NULL character at the end of string.
 *
 * \param[in] dump_buffer         A pointer to a character array.
 *
 */
void send_data(char *dump_buffer);

/*!
 * \brief Receives 'N' number of samples provided as the second argument (nsample) and place them in the provided uint8_t receive_buffer pointer.
 *
 * \param[in] nsample         	Number of bytes to be received from HOST.
 * \param[out] receive_buffer   Pointer to the received buffer of type uint8_t.
 *
 */
void receive_data(uint8_t* receive_buffer, uint16_t nsample);

/*!
 * \brief Clear the screen of the HOST's terminal connected via UART.
 *
 */
void clear_screen();

/*!
 * \brief Transmit 'N' number of IQ raw data samples (16-bit signed integer) from ADC to HOST via UART.
 *
 * \param[in] raw_adc_i         Pointer to the int16 type I ADC data array.
 * \param[in] raw_adc_q         Pointer to the int16 type Q ADC data array.
 * \param[in] nsamples          Number of 16-bit integer IQ samples to be transmitted via UART to the HOST.
 *
 */
void dumpRawIQ_int16 (int16_t  *raw_adc_i, int16_t  *raw_adc_q, uint16_t nsamples);

/*!
 * \brief Transmit 'N' number of IQ raw data samples (16-bit unsigned integer) from ADC to HOST via UART.
 *
 * \param[in] raw_adc_i         Pointer to the uint16 type I ADC data array.
 * \param[in] raw_adc_q         Pointer to the uint16 type Q ADC data array.
 * \param[in] nsamples          Number of 16-bit unsigned integer IQ samples to be transmitted via UART to the HOST.
 *
 */
void dumpRawIQ_uint16(uint16_t *raw_adc_i, uint16_t *raw_adc_q, uint16_t nsamples);

/*!
 * \brief Transmit 'N' number of IQ raw data samples (32-bit unsigned integer) from ADC to HOST via UART.
 *
 * \param[in] raw_adc_i         Pointer to the uint32 type I ADC data array.
 * \param[in] raw_adc_q         Pointer to the uint32 type Q ADC data array.
 * \param[in] nsamples          Number of 32-bit unsigned integer IQ samples to be transmitted via UART to the HOST.
 *
 */
void dumpRawIQ_uint32(uint32_t *raw_adc_i, uint32_t *raw_adc_q, uint16_t nsamples);

/*!
 * \brief Transmit a single sample of type 16-bit signed integer from MCU to HOST via UART.
 *
 * \param[in] txdata            16-bit signed integer sample to be transmitted via UART to the HOST.
 *
 */
void dump_val_int16 (int16_t txdata);

/*!
 * \brief Transmit 'N' number of data samples of type 16-bit signed integer from ADC to HOST via UART.
 *
 * \param[in] txArray           16-bit signed integer sample to be transmitted via UART to the HOST.
 * \param[in] nsamples          Number of 16-bit signed integer data samples to be transmitted via UART to the HOST.
 *
 */
void dump_arr_int16 (int16_t  *txArray, uint16_t nsamples);

/*!
 * \brief Transmit a single sample of type 16-bit unsigned integer from MCU to HOST via UART.
 *
 * \param[in] txdata            16-bit unsigned integer sample to be transmitted via UART to the HOST.
 *
 */
void dump_val_uint16(uint16_t txdata);

/*!
 * \brief Transmit 'N' number of data samples of type 16-bit unsigned integer from ADC to HOST via UART.
 *
 * \param[in] txArray           16-bit unsigned integer sample to be transmitted via UART to the HOST.
 * \param[in] nsamples          Number of 16-bit unsigned integer data samples to be transmitted via UART to the HOST.
 *
 */
void dump_arr_uint16(uint16_t *txArray, uint16_t nsamples);

/*!
 * \brief Transmit a single sample of type 32-bit unsigned integer from MCU to HOST via UART.
 *
 * \param[in] txdata            32-bit unsigned integer sample to be transmitted via UART to the HOST.
 *
 */
void dump_val_uint32(uint32_t txdata);

/*!
 * \brief Transmit 'N' number of data samples of type 32-bit unsigned integer from ADC to HOST via UART.
 *
 * \param[in] txArray           32-bit unsigned integer sample to be transmitted via UART to the HOST.
 * \param[in] nsamples          Number of 32-bit unsigned integer data samples to be transmitted via UART to the HOST.
 *
 */
void dump_arr_uint32(uint32_t *txArray, uint16_t nsamples);

/*!
 * \brief Transmit a single sample of type float from MCU to HOST via UART.
 *
 * \param[in] float_val           Floating point sample to be transmitted via UART to the HOST.
 *
 */
void dump_val_float(float float_val);

#endif /* HOSTCOMMUART_H_ */
