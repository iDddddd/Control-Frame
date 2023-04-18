//
// Created by mac on 2022/12/14.
//

#include "ManiControl.h"

MC_ctrl_t ManiControl::mc_ctrl{};
uint8_t ManiControl::rx_buff[2][BUFF_SIZE];
TASK_FLAG_t ManiControl::TaskFlag;

void ManiControl::Init() {
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

//void ManiControl::GET_Data(const volatile uint8_t *rx_buff) {


//}

void ManiControl::IT_Handle() {
    if (huart6.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    } else if (USART6->SR & UART_FLAG_IDLE) {
        static uint16_t rx_len = 0;
        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
            /* Current memory buffer used is Memory 0 */

            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            rx_len = BUFF_SIZE - hdma_usart6_rx.Instance->NDTR;

            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = BUFF_SIZE;

            //设定缓冲区1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
            if (rx_len == CONTROL_LENGTH||rx_len == MANI_LENGTH) {
                GetData(0);
            }
        } else {
            /* Current memory buffer used is Memory 1 */
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            rx_len = BUFF_SIZE - hdma_usart6_rx.Instance->NDTR;

            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = BUFF_SIZE;

            //设定缓冲区0
            DMA2_Stream1->CR &= ~(DMA_SxCR_CT);

            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if (rx_len == CONTROL_LENGTH||rx_len == MANI_LENGTH) {
                GetData(1);
            }
        }
    }
}

void ManiControl::GetData(uint8_t bufIndex) {
        if (rx_buff[bufIndex][0] == 0x7A) {
            if (rx_buff[bufIndex][1] == 0x01) {
                switch (rx_buff[bufIndex][2]) {
                    case 0x01:{
                        TaskFlag = STOP;
                        mc_ctrl.ChassisStopFlag = rx_buff[bufIndex][7];
                        StateMachine::add_function_to_state(ChassisStopTask);
                        break;
                    }
                    case 0x02:{
                        TaskFlag = MOVE;
                        mc_ctrl.x = (rx_buff[bufIndex][7] << 8u) | rx_buff[bufIndex][8];
                        mc_ctrl.y = (rx_buff[bufIndex][9] << 8u) | rx_buff[bufIndex][10];
                        StateMachine::add_function_to_state(MoveTask);
                        break;
                    }
                    case 0x03:{
                        TaskFlag = ARM;
                        mc_ctrl.ARM1.angle = (rx_buff[bufIndex][8] << 8u) | rx_buff[bufIndex][7];
                        mc_ctrl.ARM2.angle = (rx_buff[bufIndex][10] << 8u) | rx_buff[bufIndex][9];
                        mc_ctrl.ARM_Z_Flag = rx_buff[bufIndex][12];
                        StateMachine::add_function_to_state(ArmTask);
                        break;
                    }
                    case 0x04:{
                        TaskFlag = CLAW;
                        mc_ctrl.ArmServoFlag = rx_buff[bufIndex][12];
                        StateMachine::add_function_to_state(ClawTask);
                        break;
                    }
                    case 0x05:{
                        TaskFlag = TRAY;
                        mc_ctrl.TrayFlag = rx_buff[bufIndex][12];
                        StateMachine::add_function_to_state(TrayTask);
                        break;
                    }
                }
            }
        }

}


void CompleteTask(){
    uint8_t tx_message[1] = {0x01};
    HAL_UART_Transmit(&huart6,tx_message,1,3);
}

void USART6_IRQHandler() {

    ManiControl::IT_Handle();

}
