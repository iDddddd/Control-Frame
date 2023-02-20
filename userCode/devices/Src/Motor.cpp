//
// Created by LEGION on 2021/10/4.
//

#include "Motor.h"


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

void Motor::Stop() {
    stopFlag = true;

}


/*4010电机类------------------------------------------------------------------*/
/**
 * @brief Motor_4010类的构造函数
 */
FOUR_Motor_4010::FOUR_Motor_4010(COMMU_INIT_t *commu_init1, COMMU_INIT_t *commu_init2,
                                 COMMU_INIT_t *commu_init3, COMMU_INIT_t *commu_init4, MOTOR_INIT_t *motor_init1,
                                 MOTOR_INIT_t *motor_init2)
        : Motor(motor_init1, this) {
    canIDs[0] = commu_init1->_id;
    canIDs[1] = commu_init2->_id;
    canIDs[2] = commu_init3->_id;
    canIDs[3] = commu_init4->_id;
    if (motor_init1->speedPIDp) speedPIDs[0].PIDInfo = *motor_init1->speedPIDp;
    if (motor_init1->speedPIDp) speedPIDs[1].PIDInfo = *motor_init1->speedPIDp;
    if (motor_init1->speedPIDp) speedPIDs[2].PIDInfo = *motor_init2->speedPIDp;
    if (motor_init1->speedPIDp) speedPIDs[3].PIDInfo = *motor_init2->speedPIDp;
    FOURID_Bind_Rx(canIDs, RxMessage);

}

/**
 * @brief Motor_4010类的析构函数
 */
FOUR_Motor_4010::~FOUR_Motor_4010() = default;

/**
 * @brief 4010电机类的执行处理函数
 */
void FOUR_Motor_4010::Handle() {
    int16_t intensity[4];
    uint32_t id;

    for (auto canID: canIDs) {
        id = canID - 0x141;
        MotorStateUpdate(id);
        intensity[id] = IntensityCalc(id);
        if (stopFlag) {
            motor4010_intensity[id] = 0;
        } else {
            motor4010_intensity[id] = intensity[id];
        }
    }
    CANMessageGenerate();
}

/**
 * @brief 4010电机类的消息包获取任务
 */
void FOUR_Motor_4010::CANMessageGenerate() {
    if ((canQueue.rear + 1) % canQueue.MAX_MESSAGE_COUNT != canQueue.front) {

        canQueue.Data[canQueue.rear].ID = can_ID;
        canQueue.Data[canQueue.rear].canType = canType;
        canQueue.Data[canQueue.rear].message[0] = motor4010_intensity[0];
        canQueue.Data[canQueue.rear].message[1] = motor4010_intensity[0] >> 8u;
        canQueue.Data[canQueue.rear].message[2] = motor4010_intensity[1];
        canQueue.Data[canQueue.rear].message[3] = motor4010_intensity[1] >> 8u;
        canQueue.Data[canQueue.rear].message[4] = motor4010_intensity[2];
        canQueue.Data[canQueue.rear].message[5] = motor4010_intensity[2] >> 8u;
        canQueue.Data[canQueue.rear].message[6] = motor4010_intensity[3];
        canQueue.Data[canQueue.rear].message[7] = motor4010_intensity[3] >> 8u;

        canQueue.rear = (canQueue.rear + 1) % canQueue.MAX_MESSAGE_COUNT;
    }
}

/**
 * @brief 用于设置4010电机速度
 * @param _targetSpeed 目标速度
 */
void FOUR_Motor_4010::SetTargetSpeed(const float *_targetSpeed) {
    stopFlag = false;
    targetSpeed[0] = _targetSpeed[0];
    targetSpeed[1] = _targetSpeed[1];
    targetSpeed[2] = _targetSpeed[2];
    targetSpeed[3] = _targetSpeed[3];

}


/**
 * @brief 更新电机的相关状态
 * @callergraph this->Handle()
 */
void FOUR_Motor_4010::MotorStateUpdate(uint32_t id) {
    feedback[id].angle = RxMessage[id][6] | (RxMessage[id][7] << 8u);
    feedback[id].speed = RxMessage[id][4] | (RxMessage[id][5] << 8u);
    feedback[id].moment = RxMessage[id][2] | (RxMessage[id][3] << 8u);
    feedback[id].temp = RxMessage[id][1];

    switch (ctrlType) {
        case SPEED_Single: {
            state[id].speed = feedback[id].speed / reductionRatio;
        }
        case POSITION_Double: {
            state[id].speed = feedback[id].speed / reductionRatio;
            state[id].moment = feedback[id].moment;
            state[id].temperature = feedback[id].temp;
            state[id].angle = feedback[id].angle * 360 / 16384;
            float realAngle = state[id].angle;
            float thisAngle = feedback[id].angle;
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
            state[id].angle = realAngle;
            lastRead = feedback[id].angle;
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
int16_t FOUR_Motor_4010::IntensityCalc(uint32_t id) {
    int16_t intensity = 0;

    switch (ctrlType) {
        case DIRECT:
            intensity = targetSpeed[id] * 16384 / 360.0f;
            break;

        case SPEED_Single:
            intensity = speedPIDs[id].PIDCalc(targetSpeed[id], state[id].speed);
            break;

        case POSITION_Double:

            break;
    }
    return intensity;
}


/*4315电机类------------------------------------------------------------------*/
/**
 * @brief Motor_4315类的构造函数
 */
Motor_4315::Motor_4315(uint32_t _id, MOTOR_INIT_t *_init) : Motor(_init, this), RS485(_id) {

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

    rsmessage[motorIndex][5] = motor4315_angle[motorIndex];
    rsmessage[motorIndex][6] = motor4315_angle[motorIndex] >> 8u;
    rsmessage[motorIndex][7] = motor4315_angle[motorIndex] >> 16u;
    rsmessage[motorIndex][8] = motor4315_angle[motorIndex] >> 24u;

    uint16_t crc = CRC16Calc(rsmessage[motorIndex], 9);
    rsmessage[motorIndex][9] = crc;
    rsmessage[motorIndex][10] = crc >> 8u;
}

/**
 * @brief 4315电机类的执行处理函数
 */
void Motor_4315::Handle() {

    AngleCalc();
    if (stopFlag == 1) {
        motor4315_angle[rs485_ID] = (zeroAngle * 16384.0f / 360.0f);
    } else {
        motor4315_angle[rs485_ID] = (realAngle * 16384.0f / 360.0f);
    }

    RS485MessageGenerate();
}

/**
 * @brief 用于设置4315电机角度
 * @param _targetAngle 目标角度
 */
void Motor_4315::SetTargetAngle(float _targetAngle) {
    stopFlag = false;
    targetAngle = _targetAngle;
}

void Motor_4315::AngleCalc() {
    if (targetAngle - lastAngle > 180) {
        zeroAngle -= 360;
    }
    if (lastAngle - targetAngle > 180) {
        zeroAngle += 360;
    }
    lastAngle = targetAngle;
    realAngle = targetAngle + zeroAngle;
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

void Motor_4315::MotorGoBack() {

    for (int i = 0; i < 4; i++) {
        rsmessage[i][0] = 0x3E;//协议头
        rsmessage[i][1] = 0x00;//包序号
        rsmessage[i][2] = 0x01 + i; //ID
        rsmessage[i][3] = 0x52;//相对位置闭环控制命令码
        rsmessage[i][4] = 0x00;//数据包长度

        uint16_t crc = CRC16Calc(rsmessage[i], 5);
        rsmessage[i][5] = crc;
        rsmessage[i][6] = crc >> 8u;
        HAL_UART_Transmit_IT(&huart1, rsmessage[i], 7);
    }

}


