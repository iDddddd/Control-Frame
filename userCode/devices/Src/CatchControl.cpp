//
// Created by mac on 2022/12/14.
//

#include "CatchControl.h"
CC_ctrl_t CatchControl::cc_ctrl;
uint16_t CatchControl::data_length;
uint8_t CatchControl::rx_buff[2][BUFF_SIZE];

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
    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //memory buffer 1
    //内存缓冲区 1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx_buff[0]);
    //memory buffer 2
    //内存缓冲区 2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx_buff[1]);
    //data length
    //数据长度
    hdma_usart6_rx.Instance->NDTR = BUFF_SIZE;//不确定需不需要
    //enable double memory buffer
    //使能双缓冲区
    CLEAR_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_CIRC);
    //使能 DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
}
void CatchControl::GET_Data(const volatile uint8_t *buf){
    int i = 0;

    while(buf[i] != 0x01){
        i++;
    }
    if(buf[i + 1] == 0x01){
        cc_ctrl.x = (buf[i + 2] << 8u)|buf[i + 3];
        cc_ctrl.y = (buf[i + 4] << 8u)|buf[i + 5];
    }
    else if(buf[i + 1] == 0x02){
        cc_ctrl.ARM2.angle = (buf[i + 2] << 8u)|buf[i + 3];
        cc_ctrl.ARM2.speed = (buf[i + 4] << 8u)|buf[i + 5];
        cc_ctrl.ARM3.angle = (buf[i + 6] << 8u)|buf[i + 7];
    }
    else if(buf[i + 1] == 0x03){
        cc_ctrl.ArmServoFlag = buf[i + 2];
    }
    else if(buf[i + 1] == 0x04){
        cc_ctrl.ChassisStopFlag = buf[i + 2];
    }



}
void CatchControl::IT_Handle() {
    if (huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    } else if (USART3->SR & UART_FLAG_IDLE) {

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            data_length = BUFF_SIZE - hdma_usart6_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = BUFF_SIZE;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            CatchControl::GET_Data(rx_buff[0]);
        } else {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            data_length = BUFF_SIZE - hdma_usart6_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = BUFF_SIZE;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            CatchControl::GET_Data(rx_buff[1]);
        }

    }
}

void DMA2_Stream1_IRQHandler(void){

    CatchControl::IT_Handle();

}
