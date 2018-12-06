/**
 * @file uart.c
 * @date 2015-12-17
 *
 * NOTE:
 * This file is generated by DAVE. Any manual modification done to this file will be lost when the code is regenerated.
 *
 * @cond
 ***********************************************************************************************************************
 * UART v4.1.8 - Configures a USIC channel to perform transmit & receive operations using UART protocol.
 *
 * Copyright (c) 2015-2016, Infineon Technologies AG
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
 * with Infineon Technologies AG (dave@infineon.com).
 ***********************************************************************************************************************
 *
 * Change History
 * --------------
 *
 * 2015-02-16:
 *     - Initial version for DAVEv4
 *
 * 2015-06-20:
 *     - Changed the abort API name from UART_Abort_Receive to UART_AbortReceive and added return type
 *
 * 2015-06-25:
 *     - Changed protocol event handling to check event configuration
 *
 * 2015-07-06:
 *     - Changed structure name from UART_DYNAMIC_t to UART_RUNTIME_t
 *
 * 2015-07-30:
 *     - Added code for DMA and Direct mode handling
 *
 * @endcond
 *
 */

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "uart.h"

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * LOCAL DATA
 **********************************************************************************************************************/
const XMC_UART_CH_STATUS_FLAG_t uart_event_status_flags[UART_EVENT_MAX] = {
  XMC_UART_CH_STATUS_FLAG_SYNCHRONIZATION_BREAK_DETECTED,
  XMC_UART_CH_STATUS_FLAG_RECEIVER_NOISE_DETECTED,
  XMC_UART_CH_STATUS_FLAG_FORMAT_ERROR_IN_STOP_BIT_0,
  XMC_UART_CH_STATUS_FLAG_FORMAT_ERROR_IN_STOP_BIT_1,
  XMC_UART_CH_STATUS_FLAG_COLLISION_DETECTED
};
const XMC_UART_CH_EVENT_t uart_event_conf_flags[UART_EVENT_MAX] = {
  XMC_UART_CH_EVENT_SYNCHRONIZATION_BREAK,
  XMC_UART_CH_EVENT_RECEIVER_NOISE,
  XMC_UART_CH_EVENT_FORMAT_ERROR,
  XMC_UART_CH_EVENT_FORMAT_ERROR,
  XMC_UART_CH_EVENT_COLLISION
};
/***********************************************************************************************************************
 * LOCAL ROUTINES
 **********************************************************************************************************************/
#ifdef UART_TX_INTERRUPT_USED
/*Function used for handling transmit interrupt.*/
void UART_lTransmitHandler(const UART_t * const handle);
#endif
#ifdef UART_RX_INTERRUPT_USED
/*Function used for handling data reception interrupts.*/
void UART_lReceiveHandler(const UART_t * const handle);
/*Function used for reconfiguring rx FIFO while receiving data.*/
static void UART_lReconfigureRxFIFO(const UART_t * const handle, uint32_t data_size);
#endif
#ifdef UART_TX_DIRECT_USED
/*Function for transmitting data using polling.*/
static UART_STATUS_t UART_lStartTransmitPolling (const UART_t *const handle, uint8_t* data_ptr, uint32_t count);
#endif
#ifdef UART_RX_DIRECT_USED
/*Function for receiving data using polling.*/
static UART_STATUS_t UART_lStartReceivePolling (const UART_t *const handle, uint8_t* data_ptr, uint32_t count);
#endif
/*Function used for handling protocol related interrupt.*/
void UART_lProtocolHandler(const UART_t * const handle);


/**********************************************************************************************************************
 * API IMPLEMENTATION
 **********************************************************************************************************************/

/*
 * @brief API to retrieve the version of the UART APP.
 *
 * @return DAVE_APP_VERSION_t Structure containing major version, minor version
 *         and patch version.
 */
DAVE_APP_VERSION_t UART_GetAppVersion()
{
  DAVE_APP_VERSION_t version;

  version.major = UART_MAJOR_VERSION;
  version.minor = UART_MINOR_VERSION;
  version.patch = UART_PATCH_VERSION;

  return version;
}

/*
 * @brief Function to initialize the USIC Channel with GUI configured values.
 *
 * @param[in]  handle UART APP handle pointer of type UART_t*
 *
 * @return  UART_STATUS_t
 *          UART_SUCCESS: for successful UART initialization.<BR>
 *          UART_STATUS_FAILURE  : If UART initialization fails.<BR>
 *
 */
UART_STATUS_t UART_Init(const UART_t *const handle)
{
  UART_STATUS_t status = UART_STATUS_SUCCESS;
  XMC_ASSERT("UART_Init : UART APP handle invalid", (((handle != NULL)&&
      (handle->config != NULL)) &&((handle->config->fptr_uart_config != NULL)&&
      (handle->runtime != NULL))))

  /*Initialize the multiplexers required for UART configuration*/
  status = handle->config->fptr_uart_config();

  return status;
}

/*
 * @brief Common function to transmit data.
 *
 * @param[in]  handle UART APP handle pointer of type UART_t*
 * @param[in]  data_ptr Pointer to data of type uint8_t
 * @param[in]  count Number of uint8_t type bytes to be transmitted
 *
 * @return  UART_STATUS_t
 *          UART_SUCCESS: If the data is put to transmit.<BR>
 *          UART_STATUS_BUSY  : If the channel is busy.<BR>
 *          UART_STATUS_BUFFER_INVALID: Either if buffer is NULL or count is 0.<BR>
 *          UART_STATUS_MODE_MISMATCH: If the configured mode is invalid.<BR>
 *
 */
UART_STATUS_t UART_Transmit(const UART_t *const handle, uint8_t* data_ptr, uint32_t count)
{
  UART_STATUS_t ret_stat = UART_STATUS_MODE_MISMATCH;

  switch(handle->config->transmit_mode)
  {
#ifdef UART_TX_INTERRUPT_USED
  case UART_TRANSFER_MODE_INTERRUPT:
    ret_stat = UART_StartTransmitIRQ(handle, data_ptr, count);
    break;
#endif
#ifdef UART_TX_DMA_USED
  case UART_TRANSFER_MODE_DMA:
    ret_stat = UART_StartTransmitDMA(handle, data_ptr, count);
    break;
#endif
#ifdef UART_TX_DIRECT_USED
  case UART_TRANSFER_MODE_DIRECT:
    ret_stat = UART_lStartTransmitPolling(handle, data_ptr, count);
    break;
#endif
  default:
    break;
  }
  return ret_stat;
}

/*
 * @brief Common function to receive data.
 *
 * @param[in]  handle UART APP handle pointer of type UART_t*
 * @param[in]  data_ptr Pointer to data of type uint8_t
 * @param[in]  count Number of uint8_t type bytes to be received
 *
 * @return  UART_STATUS_t
 *          UART_SUCCESS: If the data is put to transmit.<BR>
 *          UART_STATUS_BUSY  : If the channel is busy.<BR>
 *          UART_STATUS_BUFFER_INVALID: Either if buffer is NULL or count is 0.<BR>
 *          UART_STATUS_MODE_MISMATCH: If the configured mode is invalid.<BR>
 *
 */
UART_STATUS_t UART_Receive(const UART_t *const handle, uint8_t* data_ptr, uint32_t count)
{
  UART_STATUS_t ret_stat = UART_STATUS_MODE_MISMATCH;

  switch(handle->config->receive_mode)
  {
#ifdef UART_RX_INTERRUPT_USED
  case UART_TRANSFER_MODE_INTERRUPT:
    ret_stat = UART_StartReceiveIRQ(handle, data_ptr, count);
    break;
#endif
#ifdef UART_RX_DMA_USED
  case UART_TRANSFER_MODE_DMA:
    ret_stat = UART_StartReceiveDMA(handle, data_ptr, count);
    break;
#endif
#ifdef UART_RX_DIRECT_USED
  case UART_TRANSFER_MODE_DIRECT:
    ret_stat = UART_lStartReceivePolling(handle, data_ptr, count);
    break;
#endif
  default:
    break;
  }
  return ret_stat;
}

#if (defined UART_TX_INTERRUPT_USED || defined UART_TX_DMA_USED)
/*
 * @brief Common function to abort ongoing transmission.
 *
 * @param[in]  handle UART APP handle pointer of type UART_t*
 *
 * @return  UART_STATUS_t
 *          UART_SUCCESS: If the transmission is aborted.<BR>
 *          UART_STATUS_FAILURE: If the channel is not transmitting.<BR>
 *          UART_STATUS_MODE_MISMATCH: If the configured mode is Direct.<BR>
 *
 */
UART_STATUS_t UART_AbortTransmit(const UART_t *const handle)
{
  UART_STATUS_t ret_stat = UART_STATUS_SUCCESS;
#ifdef UART_TX_DMA_USED
  const UART_DMA_CONFIG_t * ptr_dma_config = handle->config->transmit_dma_config;
  XMC_DMA_t * ptr_gpdma = handle->config->global_dma->dma;
#endif

  XMC_ASSERT("UART_AbortTransmit: UART APP handle invalid", ((handle != NULL)&&
            (handle->runtime != NULL)))

  /*Reset the user buffer pointer to null*/
  handle->runtime->tx_busy = false;
  handle->runtime->tx_data = NULL;

  switch(handle->config->transmit_mode)
  {
#ifdef UART_TX_INTERRUPT_USED
  case UART_TRANSFER_MODE_INTERRUPT:
    /*Disable the transmit interrupts*/
    if (handle->config->tx_fifo_size != XMC_USIC_CH_FIFO_DISABLED)
    {
      /*Disable the transmit FIFO event*/
      XMC_USIC_CH_TXFIFO_DisableEvent(handle->channel,(uint32_t)XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);
      XMC_USIC_CH_TXFIFO_Flush(handle->channel);
    }
    else
    {
      /*Disable the standard transmit event*/
      XMC_USIC_CH_DisableEvent(handle->channel, (uint32_t)XMC_USIC_CH_EVENT_TRANSMIT_BUFFER);
    }
    XMC_USIC_CH_SetTransmitBufferStatus(handle->channel, XMC_USIC_CH_TBUF_STATUS_SET_IDLE);
    break;
#endif
#ifdef UART_TX_DMA_USED
  case UART_TRANSFER_MODE_DMA:
    /*Disable the standard transmit event*/
    if (XMC_DMA_CH_IsEnabled(ptr_gpdma, ptr_dma_config->dma_channel))
    {
      XMC_DMA_CH_Disable(ptr_gpdma, ptr_dma_config->dma_channel);
      while(XMC_DMA_CH_IsEnabled(ptr_gpdma, ptr_dma_config->dma_channel)==true)
      {
      }
      XMC_USIC_CH_DisableEvent(handle->channel, (uint32_t)XMC_USIC_CH_EVENT_TRANSMIT_BUFFER);
    }
    XMC_USIC_CH_SetTransmitBufferStatus(handle->channel, XMC_USIC_CH_TBUF_STATUS_SET_IDLE);
    break;
#endif
  default:
    ret_stat = UART_STATUS_MODE_MISMATCH;
    break;
  }
  return ret_stat;
}
#endif

#if (defined UART_RX_INTERRUPT_USED || defined UART_RX_DMA_USED)
/*
 * @brief Common function to abort ongoing reception.
 *
 * @param[in]  handle UART APP handle pointer of type UART_t*
 *
 * @return  UART_STATUS_t
 *          UART_SUCCESS: If the reception is aborted.<BR>
 *          UART_STATUS_FAILURE  : If the channel is not busy.<BR>
 *          UART_STATUS_MODE_MISMATCH: If the configured mode is Direct.<BR>
 *
 */
UART_STATUS_t UART_AbortReceive(const UART_t *const handle)
{
  UART_STATUS_t ret_stat = UART_STATUS_SUCCESS;
#ifdef UART_RX_DMA_USED
  const UART_DMA_CONFIG_t * ptr_dma_config = handle->config->receive_dma_config;
  XMC_DMA_t * ptr_gpdma = handle->config->global_dma->dma;
#endif
  XMC_ASSERT("UART_AbortReceive: UART APP handle invalid", ((handle != NULL)&&
            (handle->runtime != NULL)))

  /*Reset the user buffer pointer to null*/
  handle->runtime->rx_busy = false;
  handle->runtime->rx_data = NULL;
  switch(handle->config->receive_mode)
  {
#ifdef UART_RX_INTERRUPT_USED
  case UART_TRANSFER_MODE_INTERRUPT:
    /*Disable the receive interrupts*/
    if (handle->config->rx_fifo_size != XMC_USIC_CH_FIFO_DISABLED)
    {
      XMC_USIC_CH_RXFIFO_DisableEvent(handle->channel,
            ((uint32_t)XMC_USIC_CH_RXFIFO_EVENT_CONF_STANDARD |
            (uint32_t)XMC_USIC_CH_RXFIFO_EVENT_CONF_ALTERNATE));
    }
    else
    {
      XMC_UART_CH_DisableEvent(handle->channel,
            ((uint32_t)XMC_USIC_CH_EVENT_STANDARD_RECEIVE |
            (uint32_t)XMC_USIC_CH_EVENT_ALTERNATIVE_RECEIVE));
    }
    break;
#endif
#ifdef UART_RX_DMA_USED
  case UART_TRANSFER_MODE_DMA:
    /*Disable the receive interrupts*/
    if (XMC_DMA_CH_IsEnabled(ptr_gpdma, ptr_dma_config->dma_channel))
    {
      XMC_DMA_CH_Disable(ptr_gpdma, ptr_dma_config->dma_channel);
      while(XMC_DMA_CH_IsEnabled(ptr_gpdma, ptr_dma_config->dma_channel)==true)
      {
      }
      XMC_UART_CH_DisableEvent(handle->channel,
            ((uint32_t)XMC_USIC_CH_EVENT_STANDARD_RECEIVE |
            (uint32_t)XMC_USIC_CH_EVENT_ALTERNATIVE_RECEIVE));
    }
    break;
#endif
  default:
    ret_stat = UART_STATUS_MODE_MISMATCH;
    break;
  }
  return ret_stat;
}
#endif

/*
 * @brief Changes the baudrate of UART channel.
 *
 * @param UART_t * Pointer to the UART APP handle.
 * @param baud Value of new baudrate.
 * @param oversampling Number of samples to be considered for each symbol. 16 is the standard value.
 *
 * @return UART_STATUS_t UART_STATUS_SUCCESS if baudrate changed successfully.
 *                       UART_STATUS_BUSY if the UART channel is busy.
 *
 * \par<b>Description:</b><br>
 * The function stops the channel, calculates the clock divider values to achieve the desired baudrate.
 * Sets the divider values and reconfigures the channel as per the configuration in the UI. The channel is
 * enabled at the end of configuration.
 */
UART_STATUS_t UART_SetBaudrate(const UART_t * handle, uint32_t baud, uint32_t oversampling)
{
  UART_STATUS_t ret_stat = UART_STATUS_BUSY;
  const UART_TX_CONFIG_t * ptr_tx_conf = handle->config->tx_pin_config;

  XMC_ASSERT("UART_SetBaudrate: UART APP handle invalid", ((handle != NULL)&&
            ((handle->config != NULL) && (handle->runtime != NULL))))

  if ((handle->runtime->tx_busy == false) && (handle->runtime->rx_busy == false))
  {
    /* Set UART TX pin as input pin to avoid spikes on the pin.*/
    if (handle->config->mode != UART_MODE_LOOPBACK)
    {
      XMC_GPIO_SetMode(ptr_tx_conf->port, ptr_tx_conf->pin, XMC_GPIO_MODE_INPUT_TRISTATE);
    }
    /* Stop the UART channel before changing the baudrate.*/
    if (XMC_UART_CH_Stop(handle->channel) == XMC_UART_CH_STATUS_OK)
    {
      /*Change the baudrate*/
      ret_stat = (UART_STATUS_t)XMC_UART_CH_SetBaudrate(handle->channel, baud, oversampling);
      /*Set the sample point if the baudrate is modified*/
      if (ret_stat == UART_STATUS_SUCCESS)
      {
        XMC_UART_CH_SetSamplePoint(handle->channel, (uint32_t)(oversampling >> 1U)+1U);
      }
      /*Enable UART*/
      XMC_UART_CH_Start(handle->channel);
      /* Initialize UART TX pin */
      if (handle->config->mode != UART_MODE_LOOPBACK)
      {
        XMC_GPIO_Init(ptr_tx_conf->port, ptr_tx_conf->pin, ptr_tx_conf->config);
      }
    }
    else
    {
      ret_stat = UART_STATUS_BUSY;
    }
  }
  return ret_stat;
}

#ifdef UART_TX_INTERRUPT_USED
/*
 * @brief Registers a request for transmitting data over UART channel.
 *
 * @param[in]  UART_t*  UART APP handle pointer of type UART_t
 * @param[in]  uint8_t* Pointer to data
 * @param[in]  uint32_t Total no of words to be transmitted.
 *
 * @return  UART_STATUS_t UART_STATUS_SUCCESS if the request is accepted.
 *                        UART_STATUS_BUSY if a transmission is in progress.
 * Details of function:
 * The data transmission is accomplished using transmit interrupt. User can configure
 * a callback function in the APP UI. When the data is fully transmitted, the callback
 * function will be executed. If transmit FIFO is enabled, the trigger limit is set to 0.
 * So the transmit interrupt will be generated when all the data in FIFO is moved from FIFO.
 *
 * <i>Imp Note:</i> Return value should be validated by user to ensure that the
 * request is registered.
 *
 *
 */
UART_STATUS_t UART_StartTransmitIRQ(const UART_t *const handle, uint8_t* data_ptr, uint32_t count)
{
  UART_STATUS_t ret_stat = UART_STATUS_MODE_MISMATCH;
  UART_RUNTIME_t * ptr_runtime = handle->runtime;

  XMC_ASSERT("UART_StartTransmitIRQ: UART APP handle invalid", ((handle != NULL)&&
            (handle->runtime != NULL)))

  if (handle->config->transmit_mode == UART_TRANSFER_MODE_INTERRUPT)
  {
    ret_stat = UART_STATUS_BUSY;
    if (ptr_runtime->tx_busy == false)
    {
      /*If there is no transmission in progress*/
      if ((data_ptr != NULL) && (count > 0U))
      {
        /*Obtain the address of data, size of data*/
        ptr_runtime->tx_data = data_ptr;
        ptr_runtime->tx_data_count = count;
        /*Initialize to first index and set the busy flag*/
        ptr_runtime->tx_data_index = 0U;
        ptr_runtime->tx_busy = true;

        /*Enable the transmit buffer event*/
        if (handle->config->tx_fifo_size != XMC_USIC_CH_FIFO_DISABLED)
        {
          /*Clear the transmit FIFO*/
          XMC_USIC_CH_TXFIFO_Flush(handle->channel);
          /*Enable transmit buffer interrupt*/
          XMC_USIC_CH_TXFIFO_EnableEvent(handle->channel,(uint32_t)XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);
        }
        else
        {
          XMC_USIC_CH_EnableEvent(handle->channel, (uint32_t)XMC_USIC_CH_EVENT_TRANSMIT_BUFFER);
        }
        ret_stat = UART_STATUS_SUCCESS;
        /*Trigger the transmit buffer interrupt*/
        XMC_USIC_CH_TriggerServiceRequest(handle->channel, (uint32_t)handle->config->tx_sr);
      }
      else
      {
        ret_stat = UART_STATUS_BUFFER_INVALID;
      }
    }
  }
  return ret_stat;
}
#endif

#ifdef UART_RX_INTERRUPT_USED
/*
 * @brief Registers a request to receive data over UART channel.
 *
 * @param[in]  UART_t* UART APP handle pointer of type UART_t
 * @param[in]  uint8_t*  Pointer to data array
 * @param[in]  uint32_t  Total no of bytes to be read.
 *
 * @return  UART_STATUS_t UART_STATUS_SUCCESS if the request is accepted.
 *                        UART_STATUS_BUSY if a reception is in progress.
 * Details of function:
 * This function registers the receive request by configuring the UART
 * receive FIFO/Standard buffer (depending on the user configuration). The data
 * is received asynchronously. When the requested number of data bytes are received,
 * optionally, the user configured callback function will be executed. If a callback
 * function is not configured on the APP UI, the user has to poll for the status of
 * rx_busy variable of the APP handle structure.
 *
 * <i>Imp Note:</i> Return value should be validated by user to ensure that the
 * request is registered.
 *
 *
 */
UART_STATUS_t UART_StartReceiveIRQ(const UART_t *const handle, uint8_t* data_ptr, uint32_t count)
{
  UART_STATUS_t ret_stat = UART_STATUS_MODE_MISMATCH;
  UART_RUNTIME_t * ptr_runtime = handle->runtime;

  XMC_ASSERT("UART_StartReceiveIRQ: UART APP handle invalid", ((handle != NULL)&&
            (handle->runtime != NULL)))

  if (handle->config->receive_mode == UART_TRANSFER_MODE_INTERRUPT)
  {
    ret_stat = UART_STATUS_BUSY;
    if (ptr_runtime->rx_busy == false)
    {
      /*If no active reception in progress*/
      if ((data_ptr != NULL) && (count > 0U))
      {
        /*Obtain the address of data buffer and
         * number of data bytes to be received*/
        ptr_runtime->rx_data = data_ptr;
        ptr_runtime->rx_data_count = count;
        ptr_runtime->rx_busy = true;
        ptr_runtime->rx_data_index = 0U;

        if (handle->config->rx_fifo_size != XMC_USIC_CH_FIFO_DISABLED)
        {
          /*Clear the receive FIFO, configure the trigger lime
           * and enable the receive events*/
          XMC_USIC_CH_RXFIFO_Flush(handle->channel);

          /*Configure the FIFO trigger limit based on the required data size*/
          UART_lReconfigureRxFIFO(handle, count);

          XMC_USIC_CH_RXFIFO_EnableEvent(handle->channel,
            (uint32_t)((uint32_t)XMC_USIC_CH_RXFIFO_EVENT_CONF_STANDARD |
            (uint32_t)XMC_USIC_CH_RXFIFO_EVENT_CONF_ALTERNATE));
        }
        else
        {
          XMC_USIC_CH_EnableEvent(handle->channel,
          (uint32_t)((uint32_t)XMC_USIC_CH_EVENT_STANDARD_RECEIVE | (uint32_t)XMC_USIC_CH_EVENT_ALTERNATIVE_RECEIVE));
        }
        ret_stat = UART_STATUS_SUCCESS;
      }
      else
      {
        ret_stat = UART_STATUS_BUFFER_INVALID;
      }
    }
  }
  return ret_stat;
}
#endif

#ifdef UART_TX_DMA_USED
/*
 * @brief Registers a request for transmitting data over UART channel using DMA.
 *
 * @param[in]  UART_t*  UART APP handle pointer of type UART_t
 * @param[in]  uint8_t* Pointer to data
 * @param[in]  uint32_t Total no of words to be transmitted.
 *
 * @return  UART_STATUS_t UART_STATUS_SUCCESS if the request is accepted.
 *                        UART_STATUS_BUSY if a transmission is in progress.
 * Details of function:
 * The data transmission is accomplished using a DMA channel. User can configure
 * a callback function in the APP UI. When the data is fully transmitted, the callback
 * function will be executed.
 * <i>Imp Note:</i> Return value should be validated by user to ensure that the
 * request is registered.
 *
 *
 */
UART_STATUS_t UART_StartTransmitDMA(const UART_t *const handle, uint8_t* data_ptr, uint32_t count)
{
  UART_STATUS_t ret_stat = UART_STATUS_MODE_MISMATCH;
  UART_RUNTIME_t * ptr_runtime = handle->runtime;
  const UART_DMA_CONFIG_t * ptr_dma_config = handle->config->transmit_dma_config;
  XMC_DMA_t * ptr_gpdma = handle->config->global_dma->dma;

  XMC_ASSERT("UART_StartTransmitDMA: UART APP handle invalid", (((handle != NULL)&&
            (handle->runtime != NULL))&&(handle->config != NULL)))

  if (handle->config->transmit_mode == UART_TRANSFER_MODE_DMA)
  {
    ret_stat = UART_STATUS_BUSY;
    if (ptr_runtime->tx_busy == false)
    {
      /*If there is no transmission in progress*/
      if ((data_ptr != NULL) && ((count > 0U) &&(count <= UART_DMA_MAXCOUNT)))
      {
        /*Obtain the address of data, size of data*/
        ptr_runtime->tx_data = data_ptr;
        ptr_runtime->tx_data_count = count;
        /*Initialize to first index and set the busy flag*/
        ptr_runtime->tx_data_index = 0U;
        ptr_runtime->tx_busy = true;

        /*Enable transmit event generation*/
        XMC_UART_CH_EnableEvent(handle->channel, (uint32_t)XMC_UART_CH_EVENT_TRANSMIT_BUFFER);
        ret_stat = UART_STATUS_SUCCESS;

        /*Enable DMA channel*/
        XMC_DMA_CH_SetBlockSize(ptr_gpdma, ptr_dma_config->dma_channel, count);
        XMC_DMA_CH_SetSourceAddress(ptr_gpdma, ptr_dma_config->dma_channel, (uint32_t)data_ptr);
        XMC_DMA_CH_SetDestinationAddress(ptr_gpdma, ptr_dma_config->dma_channel,
                                         (uint32_t)&(handle->channel->TBUF[0]));
        XMC_DMA_CH_Enable(ptr_gpdma, ptr_dma_config->dma_channel);
      }
      else
      {
        ret_stat = UART_STATUS_BUFFER_INVALID;
      }
    }
  }
  return ret_stat;
}
#endif

#ifdef UART_RX_DMA_USED
/*
 * @brief Registers a request to receive data over UART channel using DMA.
 *
 * @param[in]  UART_t* UART APP handle pointer of type UART_t
 * @param[in]  uint8_t*  Pointer to data array
 * @param[in]  uint32_t  Total no of bytes to be read.
 *
 * @return  UART_STATUS_t UART_STATUS_SUCCESS if the request is accepted.
 *                        UART_STATUS_BUSY if a reception is in progress.
 * Details of function:
 * This function registers the receive request by configuring the UART
 * receive Standard buffer and the DMA channel. The data
 * is received asynchronously. When the requested number of data bytes are received,
 * optionally, the user configured callback function will be executed.
 *
 * <i>Imp Note:</i> Return value should be validated by user to ensure that the
 * request is registered.
 *
 *
 */
UART_STATUS_t UART_StartReceiveDMA(const UART_t *const handle, uint8_t* data_ptr, uint32_t count)
{
  UART_STATUS_t ret_stat = UART_STATUS_MODE_MISMATCH;
  UART_RUNTIME_t * ptr_runtime = handle->runtime;
  const UART_DMA_CONFIG_t * ptr_dma_config = handle->config->receive_dma_config;
  XMC_DMA_t * ptr_gpdma = handle->config->global_dma->dma;

  XMC_ASSERT("UART_StartReceiveDMA: UART APP handle invalid", (((handle != NULL)&&
            (handle->runtime != NULL)) && (handle->config != NULL)))

  if (handle->config->receive_mode == UART_TRANSFER_MODE_DMA)
  {
    ret_stat = UART_STATUS_BUSY;
    if (ptr_runtime->rx_busy == false)
    {
      /*If no active reception in progress*/
      if ((data_ptr != NULL) && ((count > 0U) && (count <= UART_DMA_MAXCOUNT)))
      {
        /*Obtain the address of data buffer and
         * number of data bytes to be received*/
        ptr_runtime->rx_data = data_ptr;
        ptr_runtime->rx_data_count = count;
        ptr_runtime->rx_busy = true;
        ptr_runtime->rx_data_index = 0U;

        XMC_USIC_CH_EnableEvent(handle->channel,
          (uint32_t)((uint32_t)XMC_USIC_CH_EVENT_STANDARD_RECEIVE | (uint32_t)XMC_USIC_CH_EVENT_ALTERNATIVE_RECEIVE));
        ret_stat = UART_STATUS_SUCCESS;

        /*Enable DMA channel*/
        XMC_DMA_CH_SetBlockSize(ptr_gpdma, ptr_dma_config->dma_channel, count);
        XMC_DMA_CH_SetSourceAddress(ptr_gpdma, ptr_dma_config->dma_channel, (uint32_t)&(handle->channel->RBUF));
        XMC_DMA_CH_SetDestinationAddress(ptr_gpdma, ptr_dma_config->dma_channel, (uint32_t)data_ptr);
        XMC_DMA_CH_Enable(ptr_gpdma, ptr_dma_config->dma_channel);
      }
      else
      {
        ret_stat = UART_STATUS_BUFFER_INVALID;
      }
    }
  }
  return ret_stat;
}
#endif

#ifdef UART_TX_DIRECT_USED
/*
 * Polling method to transmit data.
 * @param[in] UART_t* handle UART APP handle pointer
 * @param[in] uint8_t*  Pointer to data array
 * @param[in] uint32_t number of bytes to be transmitted.
 *
 * @return UART_STATUS_t Status of transmit request handling.
 *
 * Description:
 * Transmits data by blocking the CPU until all data is sent. Transmission
 * cannot be aborted since it is blocking implementation. Based on FIFO selection,
 * either TBUF or IN register is updated with the data.
 *
 */
static UART_STATUS_t UART_lStartTransmitPolling(const UART_t *const handle, uint8_t* data_ptr, uint32_t count)
{
  UART_STATUS_t ret_stat = UART_STATUS_BUFFER_INVALID;
  uint32_t loc_index;

  XMC_ASSERT("UART_Transmit: UART APP handle invalid", (((handle != NULL)&&
            (handle->runtime != NULL))&&(handle->config != NULL)))

  if ((data_ptr != NULL) && (count > 0U))
  {
    ret_stat = UART_STATUS_BUSY;
    if (handle->runtime->tx_busy == false)
    {
      handle->runtime->tx_busy = true;
      if (handle->config->tx_fifo_size != XMC_USIC_CH_FIFO_DISABLED)
      {
        /*Clear the transmit FIFO*/
        XMC_USIC_CH_TXFIFO_Flush(handle->channel);
      }
      /*Loop through each byte*/
      for (loc_index = 0U; loc_index < count; loc_index++)
      {
        /*If FIFO is enabled, FIFO filling status should be checked
         * to avoid overflow error*/
        if (handle->config->tx_fifo_size != XMC_USIC_CH_FIFO_DISABLED)
        {
          /*Wait if transmit FIFO is full*/
          while (XMC_USIC_CH_TXFIFO_IsFull(handle->channel) == true)
          {
          }
        }
        XMC_UART_CH_Transmit(handle->channel, (uint16_t)data_ptr[loc_index]);
      }

      if (handle->config->tx_fifo_size != XMC_USIC_CH_FIFO_DISABLED)
      {
        /*Wait till FIFO is empty*/
        while (XMC_USIC_CH_TXFIFO_IsEmpty(handle->channel) == false)
        {
        }
      }
      ret_stat = UART_STATUS_SUCCESS;
      handle->runtime->tx_busy = false;
    }
  }
  return ret_stat;
}
#endif

#ifdef UART_RX_DIRECT_USED
/*
 * Polling method to receive data.
 * @param[in] UART_t* handle UART APP handle pointer
 * @param[in] uint8_t*  Pointer to data array
 * @param[in] uint32_t number of bytes to be received.
 *
 * @return UART_STATUS_t Status of receive request handling.
 *
 * Description:
 * Receives data by blocking the CPU until all data is received. Reception
 * cannot be aborted since it is blocking implementation. Based on FIFO selection,
 * either RBUF or OUT register will be read.
 *
 */
static UART_STATUS_t UART_lStartReceivePolling(const UART_t *const handle, uint8_t* data_ptr, uint32_t count)
{
  UART_STATUS_t ret_stat = UART_STATUS_BUFFER_INVALID;
  uint32_t loc_index;
  uint32_t loc_status;

  XMC_ASSERT("UART_Receive: UART APP handle invalid", ((handle != NULL)&&
            (handle->runtime != NULL)))

  if ((data_ptr != NULL) && (count > 0U))
  {
    ret_stat = UART_STATUS_BUSY;
    if (handle->runtime->rx_busy == false)
    {
      handle->runtime->rx_busy = true;
      if (handle->config->rx_fifo_size != XMC_USIC_CH_FIFO_DISABLED)
      {
        /*Clear the receive FIFO, configure the trigger lime
         * and enable the receive events*/
        XMC_USIC_CH_RXFIFO_Flush(handle->channel);
      }
      for (loc_index = 0U; loc_index < count; loc_index++)
      {
        /*If receive FIFO is configured, wait for FIFO to get data.*/
        if (handle->config->rx_fifo_size != XMC_USIC_CH_FIFO_DISABLED)
        {
          /*Wait if FIFO empty*/
          while(XMC_USIC_CH_RXFIFO_IsEmpty(handle->channel) == true)
          {
          }
        }
        else
        {
          /*Wait for RIF or AIF flag update*/
          loc_status = XMC_UART_CH_GetStatusFlag(handle->channel);
          while (!(loc_status & ((uint32_t)XMC_UART_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION |
                (uint32_t)XMC_UART_CH_STATUS_FLAG_RECEIVE_INDICATION)))
          {
            loc_status = XMC_UART_CH_GetStatusFlag(handle->channel);
          }
          /*Clear the detected event.
           * Both events should not be cleared at once, otherwise if 2 bytes are received, only
           * one byte will be read.*/
          XMC_UART_CH_ClearStatusFlag(handle->channel,
              ((uint32_t)XMC_UART_CH_STATUS_FLAG_RECEIVE_INDICATION | (uint32_t)XMC_UART_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION));
        }
        data_ptr[loc_index] = (uint8_t)XMC_UART_CH_GetReceivedData(handle->channel);
      }
      ret_stat = UART_STATUS_SUCCESS;
      handle->runtime->rx_busy = false;
    }
  }
  return ret_stat;
}
#endif

#ifdef UART_TX_INTERRUPT_USED
/*
 * Transmit interrupt handler for the APP.
 * This is a common interrupt handling function called for different instances of the APP.
 *
 *  * param[in]  handle UART APP handle pointer of type UART_t*
 *
 *  * return void
 */
void UART_lTransmitHandler(const UART_t * const handle)
{
  UART_RUNTIME_t * ptr_runtime = handle->runtime;

  if (ptr_runtime->tx_data_index < ptr_runtime->tx_data_count)
  {
    if (handle->config->tx_fifo_size != XMC_USIC_CH_FIFO_DISABLED)
    {
      /*When Transmit FIFO is enabled*/
      /*Fill the transmit FIFO */
      while (XMC_USIC_CH_TXFIFO_IsFull(handle->channel) == false)
      {
        if (ptr_runtime->tx_data_index < ptr_runtime->tx_data_count)
        {
          /*Load the FIFO byte by byte till either FIFO is full or all data is loaded*/
          XMC_UART_CH_Transmit(handle->channel,(uint16_t)ptr_runtime->tx_data[ptr_runtime->tx_data_index]);
          (ptr_runtime->tx_data_index)++;
        }
        else
        {
          break;
        }
      }
    }
    else
    {
      /*When Transmit FIFO is disabled*/
      XMC_UART_CH_Transmit(handle->channel,(uint16_t)ptr_runtime->tx_data[ptr_runtime->tx_data_index]);
      (ptr_runtime->tx_data_index)++;
    }
  }
  else
  {
    if (XMC_USIC_CH_TXFIFO_IsEmpty(handle->channel) == true)
    {
      if (handle->config->tx_fifo_size != XMC_USIC_CH_FIFO_DISABLED)
      {
        /*Disable the transmit FIFO event*/
        XMC_USIC_CH_TXFIFO_DisableEvent(handle->channel,(uint32_t)XMC_USIC_CH_TXFIFO_EVENT_CONF_STANDARD);
      }
      else
      {
        /*Disable the standard transmit event*/
        XMC_USIC_CH_DisableEvent(handle->channel, (uint32_t)XMC_USIC_CH_EVENT_TRANSMIT_BUFFER);
      }

      /*Wait for the transmit buffer to be free to ensure that all data is transmitted*/
      while (XMC_USIC_CH_GetTransmitBufferStatus(handle->channel) == XMC_USIC_CH_TBUF_STATUS_BUSY)
      {

      }
      /*All data is transmitted*/
      ptr_runtime->tx_busy = false;
      ptr_runtime->tx_data = NULL;

      if (handle->config->tx_cbhandler != NULL)
      {
        /*Execute the callback function provided in the UART APP UI*/
        handle->config->tx_cbhandler();
      }
    }
  }
}
#endif

#ifdef UART_RX_INTERRUPT_USED
/*
 * Receive interrupt handler for the APP.
 * This is a common interrupt handling function for different instances of the UART APP.
 *
 * param[in]  handle UART APP handle pointer of type UART_t*
 *
 * return void
 */
void UART_lReceiveHandler(const UART_t * const handle)
{
  UART_RUNTIME_t * ptr_runtime = handle->runtime;

  if (handle->config->rx_fifo_size != XMC_USIC_CH_FIFO_DISABLED)
  {
    /*When Receive FIFO is enabled*/
    while (XMC_USIC_CH_RXFIFO_IsEmpty(handle->channel) == false)
    {
      if (ptr_runtime->rx_data_index < ptr_runtime->rx_data_count)
      {
        /*Read all the content of Receive FIFO */
        ptr_runtime->rx_data[ptr_runtime->rx_data_index] = (uint8_t)XMC_UART_CH_GetReceivedData(handle->channel);
        (ptr_runtime->rx_data_index)++;
      }

      if (ptr_runtime->rx_data_index == ptr_runtime->rx_data_count)
      {
        /*Reception complete*/
        ptr_runtime->rx_busy = false;
        /*Disable both standard receive and alternative receive FIFO events*/
        XMC_USIC_CH_RXFIFO_DisableEvent(handle->channel,
            (uint32_t)((uint32_t)XMC_USIC_CH_RXFIFO_EVENT_CONF_STANDARD |
            (uint32_t)XMC_USIC_CH_RXFIFO_EVENT_CONF_ALTERNATE));
        if (handle->config->rx_cbhandler != NULL)
        {
          /*Execute the 'End of reception' callback function*/
          handle->config->rx_cbhandler();
        }
        break;
      }
    }
    /*Set the trigger limit if data still to be received*/
    if (ptr_runtime->rx_data_index < ptr_runtime->rx_data_count)
    {
      UART_lReconfigureRxFIFO(handle,
          (uint32_t)(ptr_runtime->rx_data_count - ptr_runtime->rx_data_index));
    }
  }
  else
  {
    /*When RxFIFO is disabled*/
    if (ptr_runtime->rx_data_index < ptr_runtime->rx_data_count)
    {
      ptr_runtime->rx_data[ptr_runtime->rx_data_index] = (uint8_t)XMC_UART_CH_GetReceivedData(handle->channel);
      (ptr_runtime->rx_data_index)++;
    }

    if (ptr_runtime->rx_data_index == ptr_runtime->rx_data_count)
    {
      /*Reception complete*/
      ptr_runtime->rx_busy = false;
      /*Disable both standard receive and alternative receive FIFO events*/
      XMC_USIC_CH_DisableEvent(handle->channel,
          (uint32_t)((uint32_t)XMC_USIC_CH_EVENT_ALTERNATIVE_RECEIVE | (uint32_t)XMC_USIC_CH_EVENT_STANDARD_RECEIVE));

      if (handle->config->rx_cbhandler != NULL)
      {
        /*Execute the 'End of reception' callback function*/
        handle->config->rx_cbhandler();
      }
    }
  }
}

/*
 * A local function to reconfigure Receive FIFO with the given size and trigger limit.
 * Size is needed because the FIFO should be disabled before changing the trigger limit by
 * clearing the FIFO size.
 *
 * param[in] UART_t * pointer to the UART APP handle
 * param[in] uint8_t  number of bytes to be received.
 *
 * return void.
 */
static void UART_lReconfigureRxFIFO(const UART_t * const handle, uint32_t data_size)
{
  uint32_t fifo_size;
  uint32_t ret_limit_val = 0U;

  /*Get FIFO size in bytes*/
  fifo_size = (uint32_t)(0x01UL << (uint8_t)(handle->config->rx_fifo_size));
  /*If data size is more than FIFO size, configure the limit to the FIFO size*/
  if (data_size < fifo_size)
  {
    ret_limit_val = (uint32_t)(data_size - 1U);
  }
  else
  {
    ret_limit_val = (uint32_t)(fifo_size - 1U);
  }
  /*Set the limit value*/
  XMC_USIC_CH_RXFIFO_SetSizeTriggerLimit(handle->channel,
        handle->config->rx_fifo_size, ret_limit_val);
}
#endif

#ifdef UART_PROTOCOL_EVENT_USED
/*
 * Protocol interrupt handling function.
 * The function is common for different instances of the UART APP.
 *
 * param[in]  handle UART APP handle pointer of type UART_t*
 *
 * return void
 */
void UART_lProtocolHandler(const UART_t * const handle)
{
  /*Protocol status value to check which event occured*/
  uint32_t psr_status = XMC_UART_CH_GetStatusFlag(handle->channel);
  /*Protocol event configuration to check which event is
   * configured for interrupt generation and hence callback*/
  uint32_t pcr_conf = handle->channel->PCR_ASCMode;
  /*Array of callback functions in the order of events*/
  const UART_cbhandler callback_arr[UART_EVENT_MAX] = {
    handle->config->sync_error_cbhandler,
    handle->config->rx_noise_error_cbhandler,
    handle->config->format_error_bit0_cbhandler,
    handle->config->format_error_bit1_cbhandler,
    handle->config->collision_error_cbhandler
  };
  UART_EVENT_t loc_index;

  for (loc_index = UART_EVENT_SYNC_BRK; loc_index < UART_EVENT_MAX; loc_index++)
  {
    /*Check if event is configured for interrupt generation and event has occured*/
    if ((pcr_conf & (uint32_t)uart_event_conf_flags[loc_index]) &&
       (psr_status & (uint32_t)uart_event_status_flags[loc_index]))
    {
      XMC_UART_CH_ClearStatusFlag(handle->channel, (uint32_t)uart_event_status_flags[(uint32_t)loc_index]);
      /*Call the callback function if it is valid*/
      if ((callback_arr[(uint32_t)loc_index] != NULL))
      {
        callback_arr[(uint32_t)loc_index]();
      }
      /*Process only one event*/
      break;
    }
  }
}
#endif
