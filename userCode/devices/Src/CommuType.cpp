//
// Created by 25396 on 2023/2/11.
//

#include "CommuType.h"

MyMap<uint32_t, uint8_t *> CAN::dict_CAN;
MyMap<uint32_t, uint8_t *> RS485::dict_RS485;
uint8_t RS485::rsmessage[4][11] = {0};
TX_QUEUE_t CAN::canQueue = {
        .front = 0,
        .rear = 0,
};
uint8_t RS485::rs485_rx_buff[2][RX_SIZE];

/*CAN类------------------------------------------------------------------*/
/**
 * @brief CAN通信的初始化，主要是CAN通信的相关配置
 */
void CAN::CANInit() {
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);//接收中断

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);//发送中断
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);//发送中断

    CAN_FilterTypeDef canFilterTypeDef;

    canFilterTypeDef.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterTypeDef.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilterTypeDef.FilterIdHigh = 0x0000;
    canFilterTypeDef.FilterIdLow = 0x0000;
    canFilterTypeDef.FilterMaskIdHigh = 0x0000;
    canFilterTypeDef.FilterMaskIdLow = 0x0000;
    canFilterTypeDef.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilterTypeDef.FilterActivation = ENABLE;
    canFilterTypeDef.FilterBank = 0;
    canFilterTypeDef.SlaveStartFilterBank = 0;

    HAL_CAN_ConfigFilter(&hcan1, &canFilterTypeDef);
    HAL_CAN_ConfigFilter(&hcan2, &canFilterTypeDef);

}

/**
 * @brief CAN类的构造函数
 */

CAN::CAN(COMMU_INIT_t *_init) {
    can_ID = _init->_id;//CAN ID
    canType = _init->canType;//CAN类型
}

/**
 * @brief CAN类的析构函数
 */
CAN::~CAN() = default;

/**
 * @brief can消息包发送任务
 * @callergraph void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 *              in Device.cpp
 */
void CAN::CANPackageSend() {
    if (canQueue.front != canQueue.rear) {
        CAN_TxHeaderTypeDef txHeaderTypeDef;
        uint32_t box = 0;//邮箱号

        txHeaderTypeDef.StdId = canQueue.Data[canQueue.front].ID;//从消息包中取出对应的ID
        txHeaderTypeDef.DLC = 0x08;//数据长度
        txHeaderTypeDef.IDE = CAN_ID_STD;//标准帧
        txHeaderTypeDef.RTR = CAN_RTR_DATA;//数据帧
        txHeaderTypeDef.TransmitGlobalTime = DISABLE;
        //根据canType选择发送的can总线
        if (canQueue.Data[canQueue.front].canType == can1) {
            HAL_CAN_AddTxMessage(&hcan1, &txHeaderTypeDef, canQueue.Data[canQueue.front].message, &box);
        } else if (canQueue.Data[canQueue.front].canType == can2) {
            /**此处注释用于Z轴电机位置模式下can消息包数据长度只有5位的情况*/
                /*if (canQueue.Data[canQueue.front].ID == 0x01) {
                    txHeaderTypeDef.DLC = 0x05;
                    HAL_CAN_AddTxMessage(&hcan2, &txHeaderTypeDef, canQueue.Data[canQueue.front].message, &box);
                } else {*/
                    HAL_CAN_AddTxMessage(&hcan2, &txHeaderTypeDef, canQueue.Data[canQueue.front].message, &box);
               // }
        }
        memset(canQueue.Data[canQueue.front].message, 0 , sizeof(canQueue.Data[canQueue.front].message));//清空消息包中的数据
        canQueue.front = (canQueue.front + 1) % MAX_MESSAGE_COUNT;//消息队列头指针后移
    }
}

/**
 * @brief can中断处理函数，用于电调返回数据的接受
 * @param hcan
 */
void CAN::Rx_Handle(CAN_HandleTypeDef *hcan) {
    uint8_t canBuf[8];
    CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, canBuf);//获取接收到的数据,完成后调用CAN中断处理函数，再次进入此函数等待接收

    memcpy(dict_CAN[rx_header.StdId], canBuf, sizeof(canBuf));//将接收到的数据拷贝到字典中,则自动进入电机的RxMessage中

}
/**
 * @brief CAN ID与RxMessage绑定
 * @param RxMessage
 */
void CAN::ID_Bind_Rx(uint8_t *RxMessage) const {
    dict_CAN.insert(can_ID, RxMessage);//将对应电机的canID与RxMessage绑定
}


/*RS485类------------------------------------------------------------------*/
/**
 * @brief RS485类的构造函数
 */
RS485::RS485(uint32_t _id) {
    rs485_ID = _id;
}

/**
 * @brief RS485类的析构函数
 */
RS485::~RS485() = default;

/**
 * @brief RS485消息包发送任务
 */
void RS485::RS485PackageSend() {
    static uint8_t rsmotorIndex = 0;
    rsmotorIndex %= 4;
    HAL_UART_Transmit_DMA(&huart1, rsmessage[rsmotorIndex], 11);

    rsmotorIndex++;
}

void RS485::RS485Init() {
    //使能 DMA 串口接收
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    //失效 DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while (hdma_usart1_rx.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }
    hdma_usart1_rx.Instance->PAR = (uint32_t) &(USART1->DR);
    //内存缓冲区 1
    hdma_usart1_rx.Instance->M0AR = (uint32_t) (rs485_rx_buff[0]);
    //内存缓冲区 2
    hdma_usart1_rx.Instance->M1AR = (uint32_t) (rs485_rx_buff[1]);
    //数据长度
    hdma_usart1_rx.Instance->NDTR = RX_SIZE;//不确定需不需要
    //使能双缓冲区
    CLEAR_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_CIRC);
    //使能 DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
}

void RS485::Rx_Handle() {
    if (huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    } else if (USART1->SR & UART_FLAG_IDLE) {
        static uint16_t rx_len = 0;
        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
            /* Current memory buffer used is Memory 0 */

            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            rx_len = RX_SIZE - hdma_usart1_rx.Instance->NDTR;

            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = RX_SIZE;

            //设定缓冲区1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            //将接收到的数据拷贝到字典中,则自动进入电机的RxMessage中
            memcpy(dict_RS485[rs485_rx_buff[0][2] - 0x01], rs485_rx_buff[0], rx_len);
        } else {
            /* Current memory buffer used is Memory 1 */
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            rx_len = RX_SIZE - hdma_usart1_rx.Instance->NDTR;

            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = RX_SIZE;

            //设定缓冲区0
            DMA2_Stream2->CR &= ~(DMA_SxCR_CT);

            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);
            memcpy(dict_RS485[rs485_rx_buff[1][2] - 0x01], rs485_rx_buff[1], rx_len);
        }
    }
}

void RS485::ID_Bind_Rx(uint8_t *RxMessage) const {
    dict_RS485.insert(rs485_ID, RxMessage);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN::Rx_Handle(hcan);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    //CAN::CANPackageSend();
}

void USART1_IRQHandler(){

    RS485::Rx_Handle();

    HAL_UART_IRQHandler(&huart1);

}
