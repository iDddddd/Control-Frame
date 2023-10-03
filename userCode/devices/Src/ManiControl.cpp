#include "ManiControl.h"

MC_ctrl_t ManiControl::mc_ctrl{};
uint8_t ManiControl::mani_rx_buff[2][BUFF_SIZE];
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
    hdma_usart6_rx.Instance->M0AR = (uint32_t) (mani_rx_buff[0]);
    //内存缓冲区 2
    hdma_usart6_rx.Instance->M1AR = (uint32_t) (mani_rx_buff[1]);
    //数据长度
    hdma_usart6_rx.Instance->NDTR = BUFF_SIZE;
    //使能双缓冲区
    CLEAR_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_CIRC);
    //使能 DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
}


void ManiControl::IT_Handle() {
    if (huart6.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    } else if (USART6->SR & UART_FLAG_IDLE) {
        static uint16_t uart6_rx_len = 0;
        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
            /* Current memory buffer used is Memory 0 */

            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            uart6_rx_len = BUFF_SIZE - hdma_usart6_rx.Instance->NDTR;

            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = BUFF_SIZE;

            //设定缓冲区1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
            /**只需关注该部分代码**/
            if (LRC_calc(mani_rx_buff[0], uart6_rx_len - 1) == mani_rx_buff[0][uart6_rx_len - 1] && (mani_rx_buff[0][0] == 0x7A)) {
                GetData(0);
            }
            memset(mani_rx_buff[0], 0, BUFF_SIZE);
            /**只需关注该部分代码**/
        } else {
            /* Current memory buffer used is Memory 1 */
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            uart6_rx_len = BUFF_SIZE - hdma_usart6_rx.Instance->NDTR;

            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = BUFF_SIZE;

            //设定缓冲区0
            DMA2_Stream1->CR &= ~(DMA_SxCR_CT);

            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
            /**只需关注该部分代码**/
            if (LRC_calc(mani_rx_buff[1], uart6_rx_len - 1) == mani_rx_buff[1][uart6_rx_len - 1] && (mani_rx_buff[1][0] == 0x7A)) {
                GetData(1);
            }
            memset(mani_rx_buff[1], 0, BUFF_SIZE);
            /**只需关注该部分代码**/
        }
    }
}

/**
 * @brief 串口数据解析
 * @param bufIndex 缓冲区索引
 * @note 可根据实际需要修改
 */
void ManiControl::GetData(uint8_t bufIndex) {
    if (mani_rx_buff[bufIndex][1] == 0x01) {
        switch (mani_rx_buff[bufIndex][2]) {
            case 0x01: {
                TaskFlag = STOP;
                mc_ctrl.ChassisStopFlag = mani_rx_buff[bufIndex][4];
                break;
            }
            case 0x02: {
                TaskFlag = MOVE_DIS;
                memcpy(&mc_ctrl.chassisDis_col.x_Dis, &mani_rx_buff[bufIndex][4], 4);
                memcpy(&mc_ctrl.chassisDis_col.y_Dis, &mani_rx_buff[bufIndex][8], 4);
                memcpy(&mc_ctrl.chassisDis_col.Theta, &mani_rx_buff[bufIndex][12], 4);
                break;
            }
            case 0x03: {
                TaskFlag = ARM;
                memcpy(&mc_ctrl.arm_col.Joint1Pos, &mani_rx_buff[bufIndex][4], 4);
                memcpy(&mc_ctrl.arm_col.Joint2Pos, &mani_rx_buff[bufIndex][8], 4);
                memcpy(&mc_ctrl.arm_col.Joint3Pos, &mani_rx_buff[bufIndex][12], 4);
                memcpy(&mc_ctrl.arm_col.Joint4Pos, &mani_rx_buff[bufIndex][16], 4);
                memcpy(&mc_ctrl.arm_col.Joint5Pos, &mani_rx_buff[bufIndex][20], 4);

                ArmJointSet(mc_ctrl.arm_col.Joint1Pos.f, mc_ctrl.arm_col.Joint2Pos.f, mc_ctrl.arm_col.Joint3Pos.f, mc_ctrl.arm_col.Joint4Pos.f, mc_ctrl.arm_col.Joint5Pos.f);
                break;
            }
            case 0x04: {
                TaskFlag = CLAW;
                mc_ctrl.ClawFlag = mani_rx_buff[bufIndex][4];

                ClawSet(mc_ctrl.ClawFlag);
                break;
            }
            case 0x05: {
                TaskFlag = TRAY;
                mc_ctrl.TrayFlag = mani_rx_buff[bufIndex][4];

                //AutoTraySet(mc_ctrl.TrayFlag);
                break;
            }
            case 0x07: {
                TaskFlag = MOVE_VEL;
                memcpy(&mc_ctrl.chassisVel_col.x_Vel, &mani_rx_buff[bufIndex][4], 4);
                memcpy(&mc_ctrl.chassisVel_col.y_Vel, &mani_rx_buff[bufIndex][8], 4);
                memcpy(&mc_ctrl.chassisVel_col.w_Vel, &mani_rx_buff[bufIndex][12], 4);
                break;
            }
            case 0x08:{
                TaskFlag = ARM_POS;
                memcpy(&mc_ctrl.arm_pos.x, &mani_rx_buff[bufIndex][4], 4);
                memcpy(&mc_ctrl.arm_pos.y, &mani_rx_buff[bufIndex][8], 4);
                memcpy(&mc_ctrl.arm_pos.z, &mani_rx_buff[bufIndex][12], 4);

                ArmPositionSet(mc_ctrl.arm_pos.x.f,mc_ctrl.arm_pos.y.f,mc_ctrl.arm_pos.z.f);

               break;
            }
        }
    }
}


void CompleteTask() {
    uint8_t tx_message[1] = {0x01};
    HAL_UART_Transmit_IT(&huart6, tx_message, 1);
}

uint8_t LRC_calc(uint8_t *data, uint8_t len) {
    uint8_t LRC = 0;
    for (uint8_t i = 0; i < len; i++) {
        LRC += data[i];
    }
    return LRC;
}

void USART6_IRQHandler() {

    ManiControl::IT_Handle();

    HAL_UART_IRQHandler(&huart6);
}

