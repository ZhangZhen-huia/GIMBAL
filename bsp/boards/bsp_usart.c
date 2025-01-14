#include "bsp_usart.h"
#include "main.h"

//extern UART_HandleTypeDef huart1;
//extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

void referee_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num, uint8_t *tx_buf, uint16_t tx_buf_num)
{
    //ʹ��DMA���ڽ���
	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
		//ʹ��DMA���ڷ���
	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

    //ʹ�ܿ����ж�
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    //ʧЧDMA
	__HAL_DMA_DISABLE(&hdma_usart6_rx);
	
	while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_usart6_rx);
	}
	
	__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

	hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
	//�ڴ滺����1
	hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //�ڴ滺����2
	hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //���ݳ���
	hdma_usart6_rx.Instance->NDTR = dma_buf_num;
	
    //ʹ��˫������
	SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    //ʹ��DMA
	__HAL_DMA_ENABLE(&hdma_usart6_rx);
	
	__HAL_DMA_DISABLE(&hdma_usart6_tx);
		
	while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_usart6_tx);
	}

	hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);
	
	hdma_usart6_tx.Instance->M0AR = (uint32_t)(tx_buf);
	
	hdma_usart6_tx.Instance->NDTR = tx_buf_num;
	
	__HAL_DMA_ENABLE(&hdma_usart6_tx);

}
//DMA����һ������
void Referee_DMA_TX( uint8_t * data,uint16_t tx_buf_num)
{

	__HAL_DMA_DISABLE(&hdma_usart6_tx);
	
	while( hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_usart6_tx);
	}
	
	__HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);
	
	
	hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
	__HAL_DMA_SET_COUNTER(&hdma_usart6_tx, tx_buf_num);
		
	__HAL_DMA_ENABLE(&hdma_usart6_tx);
}


