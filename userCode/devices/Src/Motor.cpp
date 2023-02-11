//
// Created by LEGION on 2021/10/4.
//

#include "Motor.h"

uint8_t CAN::canmessage[8] = {0};
uint8_t RS485::rsmessage[4][11] = {0};
int16_t Motor_4315::motor4315_intensity[8];
int16_t Motor_4010::motor4010_intensity[8];
std::map<uint16_t, uint8_t *> CAN::dict;
Motor_Object_t *Motor::head_;

/*Motor类----------------------------------------------------------------*/
/**
 * @brief Motor类的构造函数
 * @param _init 类的初始化结构体指针
 */
Motor::Motor(MOTOR_INIT_t *_init, Motor *motor) {
    deviceType = MOTOR;

    if (_init->speedPIDp) speedPID.PIDInfo = *_init->speedPIDp;
    if (_init->anglePIDp) anglePID.PIDInfo = *_init->anglePIDp;
    reductionRatio = _init->reductionRatio;

    auto *new_object = new Motor_Object_t();
    new_object->motor_object = motor;
    new_object->next = head_;
    head_ = new_object;

}

/**
* @brief Motor类的析构函数
*/
Motor::~Motor() = default;

void Motor::ErrorHandle() {}

void Motor::MotorsHandle() {

    Motor_Object_t *current = head_;
    while (current) {
        current->motor_object->Handle();
        current = current->next;
    }

}

/*CAN类------------------------------------------------------------------*/
/**
 * @brief CAN通信的初始化，主要是CAN通信的相关配置
 */
void CAN::CANInit() {
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

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
CAN::CAN(COMMU_INIT_t *_init, uint8_t *RxMessage) {
    can_ID = _init->_id;
    ctrlType = _init->ctrlType;
    dict.insert(std::pair<uint16_t, uint8_t *>(can_ID, RxMessage));

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
    CAN_TxHeaderTypeDef txHeaderTypeDef;
    uint32_t box;

    txHeaderTypeDef.StdId = 0x280;
    txHeaderTypeDef.DLC = 0x08;
    txHeaderTypeDef.IDE = CAN_ID_STD;
    txHeaderTypeDef.RTR = CAN_RTR_DATA;
    txHeaderTypeDef.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(&hcan1, &txHeaderTypeDef, canmessage, &box);
}

/**
 * @brief can中断处理函数，用于电调返回数据的接受
 * @param hcan
 */
void CAN::Rx_Handle(CAN_HandleTypeDef *hcan) {
    uint8_t canBuf[8];
    CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, canBuf);
    if (hcan == &hcan1) {
        memcpy(dict[rx_header.StdId], canBuf, sizeof(canBuf));
    }

}

/*RS485类------------------------------------------------------------------*/
/**
 * @brief RS485类的构造函数
 */
RS485::RS485(uint16_t _id) {
    rs485_ID = _id;
    ctrlType = DIRECT;
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
    HAL_UART_Transmit_IT(&huart1, rsmessage[rsmotorIndex], 11);

    rsmotorIndex++;
}
/*4010电机类------------------------------------------------------------------*/
/**
 * @brief Motor_4010类的构造函数
 */
Motor_4010::Motor_4010(COMMU_INIT_t *commu_init, MOTOR_INIT_t *motor_init) : Motor(motor_init, this),
                                                                             CAN(commu_init, RxMessage) {


}

/**
 * @brief Motor_4010类的析构函数
 */
Motor_4010::~Motor_4010() = default;

/**
 * @brief 4010电机类的执行处理函数
 */
void Motor_4010::Handle() {
    MotorStateUpdate();

    int16_t intensity = IntensityCalc();

    if (stopFlag == 1) {
        motor4010_intensity[can_ID - 0x141] = 0;
    } else {
        motor4010_intensity[can_ID - 0x141] = intensity;
    }

    CANMessageGenerate();
}

/**
 * @brief 4010电机类的消息包获取任务
 */
void Motor_4010::CANMessageGenerate() {

    canmessage[0] = motor4010_intensity[0];
    canmessage[1] = motor4010_intensity[0] >> 8u;
    canmessage[2] = motor4010_intensity[1];
    canmessage[3] = motor4010_intensity[1] >> 8u;
    canmessage[4] = motor4010_intensity[2];
    canmessage[5] = motor4010_intensity[2] >> 8u;
    canmessage[6] = motor4010_intensity[3];
    canmessage[7] = motor4010_intensity[3] >> 8u;

}

/**
 * @brief 用于设置4010电机速度
 * @param _targetSpeed 目标速度
 */
void Motor_4010::SetTargetSpeed(float _targetSpeed) {
    stopFlag = false;
    targetSpeed = _targetSpeed;
}

/**
 * @brief 用于设置4010电机角度
 * @param _targetAngle 目标角度
 */
void Motor_4010::SetTargetAngle(float _targetAngle) {
    stopFlag = false;
    targetSpeed = _targetAngle;
}

/**
 * @brief 用于使4010电机停止
 */
void Motor_4010::Stop() {
    stopFlag = true;
}

/**
 * @brief 更新电机的相关状态
 * @callergraph this->Handle()
 */
void Motor_4010::MotorStateUpdate() {

    feedback.angle = RxMessage[6] | (RxMessage[7] << 8u);
    feedback.speed = RxMessage[4] | (RxMessage[5] << 8u);
    feedback.moment = RxMessage[2] | (RxMessage[3] << 8u);
    feedback.temp = RxMessage[1];

    switch (ctrlType) {
        case SPEED_Single: {
            state.speed = feedback.speed / reductionRatio;
        }
        case POSITION_Double: {
            state.speed = feedback.speed / reductionRatio;
            state.moment = feedback.moment;
            state.temperature = feedback.temp;
            state.angle = feedback.angle * 360 / 16384;
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
int16_t Motor_4010::IntensityCalc() {
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


/*4315电机类------------------------------------------------------------------*/
/**
 * @brief Motor_4315类的构造函数
 */
Motor_4315::Motor_4315(uint16_t _id, MOTOR_INIT_t *_init) : Motor(_init, this), RS485(_id) {

}

/**
 * @brief Motor_4315类的析构函数
 */
Motor_4315::~Motor_4315() = default;

/**
 * @brief 4315电机消息包获取任务
 */
void Motor_4315::RS485MessageGenerate() {
    int motorIndex = rs485_ID;

    rsmessage[motorIndex][0] = 0x3E;//协议头
    rsmessage[motorIndex][1] = 0x00;//包序号
    rsmessage[motorIndex][2] = 0x01 + motorIndex; //ID
    rsmessage[motorIndex][3] = 0x55;//相对位置闭环控制命令码
    rsmessage[motorIndex][4] = 0x04;//数据包长度

    uint32_t tmp = motor4315_intensity[motorIndex];
    rsmessage[motorIndex][8] = tmp >> 24u;
    rsmessage[motorIndex][7] = tmp >> 16u;
    rsmessage[motorIndex][6] = tmp >> 8u;
    rsmessage[motorIndex][5] = tmp;

    uint16_t crc = CRC16Calc(rsmessage[motorIndex], 9);
    rsmessage[motorIndex][9] = crc;
    rsmessage[motorIndex][10] = crc >> 8u;
}

/**
 * @brief 4315电机类的执行处理函数
 */
void Motor_4315::Handle() {

    int16_t intensity = targetAngle;

    if (stopFlag == 1) {
        motor4315_intensity[rs485_ID] = 0;
    } else {
        motor4315_intensity[rs485_ID] = intensity;
    }

    RS485MessageGenerate();
}

/**
 * @brief 用于设置4315电机角度
 * @param _targetAngle 目标角度
 */
void Motor_4315::SetTargetAngle(float _targetAngle) {
    stopFlag = false;
    targetAngle = _targetAngle * 16384 / 360.0f;
}

/**
 * @brief 用于使4315电机停止
 */
void Motor_4315::Stop() {
    stopFlag = true;
}

uint16_t Motor_4315::CRC16Calc(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xffff;        // Initial value
    while (length--) {
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


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN::Rx_Handle(hcan);
}

