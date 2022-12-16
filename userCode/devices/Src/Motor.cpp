//
// Created by LEGION on 2021/10/4.
//

#include "Motor.h"

Motor* Motor::motorPtrs[2][8] = {nullptr};
int16_t Motor::motor_intensity[2][8] = {0};
uint32_t Motor::motor_IDs[2];
uint8_t Motor::rsmessage[4][11] = {0};
//static int16_t feedbackangle = 0;


/**
 * @brief 电机类的初始化，主要是CAN通信的相关配置
 * @callergraph int main(void) in Device.cpp
 */
void Motor::Init() {
    HAL_CAN_Start(&hcan1);
    //HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_TX_MAILBOX_EMPTY);
    //HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

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
    //HAL_CAN_ConfigFilter(&hcan2, &canFilterTypeDef);
}
/**
 * @brief CRC16_MODBUS校验
 *
 */
uint16_t Motor::CRC16Calc(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xffff;        // Initial value
    while(length--) {
        crc ^= *data++;            // crc ^= *data; data++;
        for (uint8_t i = 0; i < 8; ++i) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;        // 0xA001 = reverse 0x8005
            else
                crc = (crc >> 1);
        }
    }
    return crc;
}

void Motor::RS485MessageGenerate() {
    int motorIndex = GET_MOTOR_POS(deviceID);

    rsmessage[motorIndex][0] = 0x3E;//协议头
    rsmessage[motorIndex][1] = 0x00;//包序号
    rsmessage[motorIndex][2] = 0x01 + motorIndex; //ID
    rsmessage[motorIndex][3] = 0x55;//相对位置闭环控制命令码
    rsmessage[motorIndex][4] = 0x04;//数据包长度

    uint32_t tmp = motor_intensity[1][motorIndex] ;
    rsmessage[motorIndex][8] = tmp >> 24u;
    rsmessage[motorIndex][7] = tmp >> 16u;
    rsmessage[motorIndex][6] = tmp >> 8u;
    rsmessage[motorIndex][5] = tmp;

    uint16_t crc = CRC16Calc(rsmessage[motorIndex], 9);
    rsmessage[motorIndex][9] = crc;
    rsmessage[motorIndex][10] = crc >> 8u;
}

/**
 * @brief rs485消息包发送任务
 */
void Motor::RS485PackageSend() {
    static uint8_t rsmotorIndex = 0;
    rsmotorIndex %= 4;
    HAL_UART_Transmit_IT(&huart1, rsmessage[rsmotorIndex], 11);

    rsmotorIndex++;

}


/**
 * @brief can消息包发送任务
 * @callergraph void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 *              in Device.cpp
 */
void Motor::CANPackageSend() {


    CAN_TxHeaderTypeDef txHeaderTypeDef;
    static uint8_t canmessage[8] = {0};

    txHeaderTypeDef.StdId = 0x280;
    txHeaderTypeDef.DLC = 0x08;
    txHeaderTypeDef.IDE = CAN_ID_STD;
    txHeaderTypeDef.RTR = CAN_RTR_DATA;
    txHeaderTypeDef.TransmitGlobalTime = DISABLE;

    canmessage[0] = motor_intensity[0][0];
    canmessage[1] = motor_intensity[0][0] >> 8u;
    canmessage[2] = motor_intensity[0][1] ;
    canmessage[3] = motor_intensity[0][1] >> 8u;
    canmessage[4] = motor_intensity[0][2];
    canmessage[5] = motor_intensity[0][2] >> 8u;
    canmessage[6] = motor_intensity[0][3];
    canmessage[7] = motor_intensity[0][3] >> 8u;

    HAL_CAN_AddTxMessage(&hcan1, &txHeaderTypeDef,canmessage,0);
}
/**
 * @brief 机械臂4010电机发送任务
 */
void Motor::ARMPackageSend(){
    CAN_TxHeaderTypeDef txHeaderTypeDef;
    static uint8_t armmessage[8] = {0};

    txHeaderTypeDef.StdId = 0x142;
    txHeaderTypeDef.DLC = 0x08;
    txHeaderTypeDef.IDE = CAN_ID_STD;
    txHeaderTypeDef.RTR = CAN_RTR_DATA;
    txHeaderTypeDef.TransmitGlobalTime = DISABLE;

    armmessage[0] = 0xA1;
    armmessage[1] = 0x00;
    armmessage[2] = 0x00;
    armmessage[3] = 0x00;
    armmessage[4] = motor_intensity[0][4];
    armmessage[5] = motor_intensity[0][4] >> 8u;
    armmessage[6] = 0x00;
    armmessage[7] = 0x00;

    HAL_CAN_AddTxMessage(&hcan2, &txHeaderTypeDef,armmessage,0);
}


/**
 * @brief Motor类的构造函数
 * @param _init 类的初始化结构体指针
 */
Motor::Motor(uint32_t _id, MOTOR_INIT_t* _init){
    _init->_motorID = _id;
    deviceID = _init->_motorID;
    deviceType = MOTOR;

    uint8_t motorPos = GET_MOTOR_POS(_init->_motorID);
    switch(_init->commuType) {
        case CAN:
            motorPtrs[0][motorPos % 8] = this;
            //TODO 检查电机ID冲突
            motor_IDs[0] |= _init->_motorID;

            break;

        case RS485:
            motorPtrs[1][motorPos % 8] = this;
            motor_IDs[1] |= _init->_motorID;

            break;
    }

    memset(&feedback, 0, sizeof(C6x0Rx_t));
    if(_init->speedPIDp) speedPID.PIDInfo = *_init->speedPIDp;
    if(_init->anglePIDp) anglePID.PIDInfo = *_init->anglePIDp;
    ctrlType = _init->ctrlType;
    commuType = _init->commuType;
    reductionRatio = _init->reductionRatio;
}

/**
 * @brief 电机类的析构函数
 */
Motor::~Motor(){
    motor_IDs[0] &= (~deviceID);
    motor_IDs[1] &= (~deviceID);
}
/**
 * @brief 电机类的执行处理函数，囊括每个电机的主要处理任务。此函数需定时执行，否则对应电机无法正常工作
 */
void Motor::Handle(){
    MotorStateUpdate();

    int16_t intensity = IntensityCalc();

    if (stopFlag == 1){
        motor_intensity[commuType][GET_MOTOR_POS(deviceID)] = 0;
    }else {
        motor_intensity[commuType][GET_MOTOR_POS(deviceID)] = intensity;
    }

    switch(commuType) {
        case CAN:
            break;

        case RS485:
            RS485MessageGenerate();
            break;
    }
}

/**
 * @brief 电机的中断处理函数，用于电调返回数据的接受
 * @param hcan
 */
void Motor::IT_Handle(CAN_HandleTypeDef *hcan) {
    uint8_t canBuf[8];
    CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, canBuf);
    uint8_t canPos,motorPos;
    if(hcan == &hcan1){
        canPos = 0;
        motorPos = rx_header.StdId - 0x141;
        motorPtrs[canPos][motorPos]->feedback.angle = canBuf[6] | (canBuf[7]<<8u);
        motorPtrs[canPos][motorPos]->feedback.speed = canBuf[4] | (canBuf[5]<<8u);
        motorPtrs[canPos][motorPos]->feedback.moment = canBuf[2] | (canBuf[3]<<8u);
        motorPtrs[canPos][motorPos]->feedback.temp = canBuf[1];
    }else {
        canPos = 0;
        motorPtrs[canPos][4]->feedback.angle = canBuf[6] | (canBuf[7]<<8u);
        motorPtrs[canPos][4]->feedback.speed = canBuf[4] | (canBuf[5]<<8u);
        motorPtrs[canPos][4]->feedback.moment = canBuf[2] | (canBuf[3]<<8u);
        motorPtrs[canPos][4]->feedback.temp = canBuf[1];
    }

}

/**
 * @brief 用于设置4010电机速度，只用当电机控制类型对应时才有效
 * @param _targetSpeed 目标速度
 */
void Motor::SetTargetSpeed(float _targetSpeed) {
    stopFlag = 0;
    targetSpeed = _targetSpeed;
}
/**
 * @brief 用于设置4010电机角度，只用当电机控制类型对应时才有效
 * @param _targetAngle 目标角度
 */
void Motor::SetTargetAngle(float _targetAngle) {
    stopFlag = false;
    targetAngle = _targetAngle;
}

void Motor::Stop() {
    stopFlag = true;
}

void Motor::ErrorHandle() {

}
/**
 * @brief 更新电机的相关状态
 * @callergraph this->Handle()
 */
void Motor::MotorStateUpdate() {

    switch(ctrlType) {
        case SPEED_Single: {
            state.speed = feedback.speed / reductionRatio;
        }

        case POSITION_Double: {
            state.speed = feedback.speed / reductionRatio;
            state.moment = feedback.moment;
            state.temperature = feedback.temp;

            float realAngle = state.angle;
            float thisAngle = feedback.angle;
            static int32_t lastRead = 0;
            if (thisAngle <= lastRead) {
                if (lastRead - thisAngle > 8000)
                    realAngle += (thisAngle + 16384 - lastRead) * 360.0f / 16384.0f / reductionRatio;
                else
                    realAngle -= (lastRead - thisAngle) * 360.0f / 16384.0f / reductionRatio;
            } else {
                if (thisAngle - lastRead > 8000)
                    realAngle -= (lastRead + 16384 - thisAngle) * 360.0f / 16384.0f / reductionRatio;
                else
                    realAngle += (thisAngle - lastRead) * 360.0f / 16384.0f / reductionRatio;
            }

            state.angle = realAngle;
            lastRead = feedback.angle;
            break;
        }

        case DIRECT:

            break;
    }
}
/**
 * @brief 计算电机实际控制电流
 * @return 控制电流值
 */
int16_t Motor::IntensityCalc() {
    int16_t intensity;
    switch (ctrlType) {
        case DIRECT:
            intensity = targetAngle * 16384 / 360.0f;
            break;

        case SPEED_Single:
            intensity = speedPID.PIDCalc(targetSpeed, state.speed);
            break;

        case POSITION_Double:
            float _targetSpeed = anglePID.PIDCalc(targetAngle, state.angle);
            intensity = speedPID.PIDCalc(_targetSpeed, state.speed);
            break;

    }
    return intensity;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    Motor::IT_Handle(hcan);
}


