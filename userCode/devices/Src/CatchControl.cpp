//
// Created by mac on 2022/12/14.
//

#include "CatchControl.h"

CC_ctrl_t CatchControl::cc_ctrl;
uint16_t CatchControl::data_length;
uint8_t CatchControl::rx_buff[2][BUFF_SIZE];

void CatchControl::Init() {
    //使能 DMA 串口接收
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    //失效 DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while (hdma_usart6_rx.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }
    hdma_usart6_rx.Instance->PAR = (uint32_t) &(USART6->DR);
    //内存缓冲区 1
    hdma_usart6_rx.Instance->M0AR = (uint32_t) (rx_buff[0]);
    //内存缓冲区 2
    hdma_usart6_rx.Instance->M1AR = (uint32_t) (rx_buff[1]);
    //数据长度
    hdma_usart6_rx.Instance->NDTR = BUFF_SIZE;//不确定需不需要
    //使能双缓冲区
    CLEAR_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_CIRC);
    //使能 DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
}

void CatchControl::GET_Data(const volatile uint8_t *buf) {

    if (buf[0] == 0x7A) {
        if (buf[1] == 0x01) {
            if (buf[2] == 0x02) {
                cc_ctrl.x = (buf[7] << 8u) | buf[8];
                cc_ctrl.y = (buf[9] << 8u) | buf[10];
            } else if (buf[2] == 0x03) {
                cc_ctrl.ARM2.angle = (buf[7] << 8u) | buf[8];
                cc_ctrl.ARM2.speed = (buf[9] << 8u) | buf[10];
                cc_ctrl.ARM3.angle = (buf[11] << 8u) | buf[12];
            } else if (buf[2] == 0x04) {
                cc_ctrl.ArmServoFlag = buf[7];
            } else if (buf[2] == 0x01) {
                cc_ctrl.ChassisStopFlag = buf[7];
            }
        }
    }
}

void CatchControl::IT_Handle() {
    if (huart6.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    } else if (USART6->SR & UART_FLAG_IDLE) {

        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
            /* Current memory buffer used is Memory 0 */

            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            data_length = BUFF_SIZE - hdma_usart6_rx.Instance->NDTR;

            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = BUFF_SIZE;

            //设定缓冲区1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
            if (data_length == CONTROL_LENGTH) {
                CatchControl::GET_Data(rx_buff[0]);
            }
        } else {
            /* Current memory buffer used is Memory 1 */
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            data_length = BUFF_SIZE - hdma_usart6_rx.Instance->NDTR;

            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = BUFF_SIZE;

            //设定缓冲区0
            DMA2_Stream1->CR &= ~(DMA_SxCR_CT);

            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if (data_length == CONTROL_LENGTH) {
                CatchControl::GET_Data(rx_buff[1]);
            }
        }

    }
}

void USART6_IRQHandler() {

    CatchControl::IT_Handle();

}
