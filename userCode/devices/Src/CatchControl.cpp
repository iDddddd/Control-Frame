//
// Created by David9686 on 2022/12/10.
//

#include "CatchControl.h"

CC_ctrl_t CatchControl::cc_ctrl;
uint8_t CatchControl::data_length;

void CatchControl::Init() {
    //enable the DMA transfer for the receiver request
    //使能 DMA 串口接收
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    //disable DMA
    //失效 DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }
    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //data length
    //数据长度
    hdma_usart3_rx.Instance->NDTR = BUFF_SIZE;//不确定需不需要

    //使能 DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
}
void CatchControl::GET_Data(){

}
void CatchControl::IT_Handle() {
    if(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET){

        __HAL_UART_CLEAR_IDLEFLAG(&huart6);
        HAL_UART_DMAStop(&huart6);
        data_length = BUFF_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
        HAL_UART_Receive_DMA(&huart6, rx_buff, BUFF_SIZE);
        CatchControl::GET_Data();
    }

}
void URAT3_IRQHandler(void){

    CatchControl::IT_Handle();

}