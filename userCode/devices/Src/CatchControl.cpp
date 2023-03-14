//
// Created by mac on 2022/12/14.
//

#include "CatchControl.h"

CC_ctrl_t CatchControl::cc_ctrl{};
uint16_t CatchControl::data_length;
uint8_t CatchControl::rx_buff[2][BUFF_SIZE];
uint8_t tx_buff[2] = {0x01,0x02};
TASK_FLAG_t CatchControl::TaskFlag;

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

//void CatchControl::GET_Data(const volatile uint8_t *rx_buff) {


//}

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

            GetData(0);

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

            GetData(1);

        }
    }
}

void CatchControl::GetData(uint8_t bufIndex) {
    int i = 0;
    while (i < 14) {
        if (rx_buff[bufIndex][i + 0] == 0x7A) {
            if (rx_buff[bufIndex][i + 1] == 0x01) {
                switch (rx_buff[bufIndex][i + 2]) {
                    case 0x01:{
                        TaskFlag = STOP;
                        cc_ctrl.ChassisStopFlag = rx_buff[bufIndex][i + 7];
                        break;
                    }
                    case 0x02:{
                        TaskFlag = MOVE;
                        cc_ctrl.x = (rx_buff[bufIndex][i + 7] << 8u) | rx_buff[bufIndex][i + 8];
                        cc_ctrl.y = (rx_buff[bufIndex][i + 9] << 8u) | rx_buff[bufIndex][i + 10];

                        break;
                    }
                    case 0x03:{
                        TaskFlag = ARM;
                        cc_ctrl.ARM1.angle = (rx_buff[bufIndex][i + 8] << 8u) | rx_buff[bufIndex][i + 7];
                        cc_ctrl.ARM2.angle = (rx_buff[bufIndex][i + 10] << 8u) | rx_buff[bufIndex][i + 9];
                        cc_ctrl.ARM_Z_Flag = rx_buff[bufIndex][i + 12];
                        break;
                    }
                    case 0x04:{
                        TaskFlag = CLAW;
                        cc_ctrl.ArmServoFlag = rx_buff[bufIndex][i + 7];
                        break;
                    }
                    case 0x05:{
                        TaskFlag = TRAY;
                        cc_ctrl.TrayFlag = rx_buff[bufIndex][i + 7];
                        break;
                    }
                }
            }
        }
        i++;
    }

}

void CatchControl::AutoTask() {
    switch (TaskFlag) {
        case STOP:
            AutoChassisStop();
            break;
        case MOVE:
            AutoChassisSet(cc_ctrl.x,cc_ctrl.y);
            break;
        case ARM:
            AutoArmSet(cc_ctrl.ARM1.angle,cc_ctrl.ARM2.angle,cc_ctrl.ARM_Z_Flag);
            break;
        case TRAY:
            AutoTraySet(cc_ctrl.TrayFlag);
            break;
        case CLAW:
            AutoClawSet(cc_ctrl.ArmServoFlag);
            break;
    }
}


void USART6_IRQHandler() {

    CatchControl::IT_Handle();

}
