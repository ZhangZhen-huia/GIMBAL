#ifndef BSP_USART_H
#define BSP_USART_H
#include "struct_typedef.h"

void referee_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num, uint8_t *tx_buf, uint16_t tx_buf_num);//����ϵͳ�ײ��ʼ��
void Referee_DMA_TX(uint8_t *data,uint16_t tx_buf_num);//�Բ���ϵͳ����һ��DMA����

#endif
