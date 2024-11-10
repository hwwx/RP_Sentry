/**
 * @file        drv_uart.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        15-August-2020
 * @brief       UART Driver Package(Based on HAL).
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_uart.h"
#include "string.h"

extern UART_HandleTypeDef huart2;

// extern rc_t               rc_structure;

/* Private macro -------------------------------------------------------------*/

#define USART2_RX_BUF_LEN 200
/* Private function prototypes -----------------------------------------------*/
__WEAK void USART2_rxDataHandler(uint8_t *rxBuf);
__WEAK void USART4_rxDataHandler(uint8_t *rxBuf);
__WEAK void USART5_rxDataHandler(uint8_t *rxBuf);
__WEAK void USART1_rxDataHandler(uint8_t *rxBuf);

static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma);
static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma);
static void uart_rx_idle_callback(UART_HandleTypeDef *huart);
static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma,
													 uint32_t SrcAddress,
													 uint32_t DstAddress,
													 uint32_t SecondMemAddress,
													 uint32_t DataLength);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t usart2_dma_rxbuf[USART2_RX_BUF_LEN];

/* Exported variables --------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief   clear idle it flag after uart receive a frame data
 * @param   uart IRQHandler id
 * @usage   call in DRV_UART_IRQHandler() function
 */
static void uart_rx_idle_callback(UART_HandleTypeDef *huart)
{
	/* clear idle it flag avoid idle interrupt all the time */
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	/* handle received data in idle interrupt */

	if (huart == &huart2)
	{
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);
		/* handle dbus data dbus_buf from DMA */
		USART1_rxDataHandler(usart2_dma_rxbuf);
		memset(usart2_dma_rxbuf, 0, USART2_RX_BUF_LEN);
		/* restart dma transmission */
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

static HAL_StatusTypeDef DMA_Start(DMA_HandleTypeDef *hdma,
								   uint32_t SrcAddress,
								   uint32_t DstAddress,
								   uint32_t DataLength)
{
	HAL_StatusTypeDef status = HAL_OK;

	/* Process locked */
	__HAL_LOCK(hdma);
	if (HAL_DMA_STATE_READY == hdma->State)
	{
		/* Change DMA peripheral state */
		hdma->State = HAL_DMA_STATE_BUSY;

		/* Initialize the error code */
		hdma->ErrorCode = HAL_DMA_ERROR_NONE;

		/* Configure the source, destination address and the data length */
		/* Clear DBM bit */
		//		hdma->Instance->CCR &= (uint32_t)(~DMA_SxCR_DBM);

		/* Configure DMA Stream data length */
		hdma->Instance->CNDTR = DataLength;

		/* Memory to Peripheral */
		if ((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
		{
			/* Configure DMA Stream destination address */
			hdma->Instance->CPAR = DstAddress; // 外设地址

			/* Configure DMA Stream source address */
			hdma->Instance->CMAR = SrcAddress; // 储存器
		}
		/* Peripheral to Memory */
		else
		{
			/* Configure DMA Stream source address */
			hdma->Instance->CPAR = SrcAddress;

			/* Configure DMA Stream destination address */
			hdma->Instance->CMAR = DstAddress;
		}

		/* Enable the Peripheral */
		__HAL_DMA_ENABLE(hdma);
	}
	else
	{
		/* Process unlocked */
		__HAL_UNLOCK(hdma);

		/* Return error status */
		status = HAL_BUSY;
	}
	return status;
}
/* Exported functions --------------------------------------------------------*/
/**
 * @brief   callback this function when uart interrupt
 * @param   uart IRQHandler id
 * @usage   call in uart handler function USARTx_IRQHandler()
 */
void DRV_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	// 判断是否为空闲中断
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart);
	}
}

/**
 * @description: 
 * @param {Enable the DMA transfer for the receiver request} SET_BIT
 * @param {	 } USART2_RX_BUF_LEN
 * @return {*}
 */
void USART2_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart2);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

	// Enable the DMA transfer for the receiver request
	SET_BIT(huart2.Instance->CR3, USART_CR3_DMAR);

	DMA_Start(huart2.hdmarx,
			  (uint32_t)&huart2.Instance->RDR,
			  (uint32_t)usart2_dma_rxbuf,
			  USART2_RX_BUF_LEN);
}

void UART2_SendData(uint8_t *Data, uint16_t Size)
{
	HAL_UART_Transmit(&huart2, Data, Size, 5);
}
/* rxData Handler [Weak] functions -------------------------------------------*/
/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 USART2 处理协议
 */
__WEAK void USART1_rxDataHandler(uint8_t *rxBuf)
{
}
