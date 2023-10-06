#include "ChassisMotor.h"



/*4315电机类------------------------------------------------------------------*/
/**
 * @brief Motor_4315类的构造函数
 */
Motor_4315::Motor_4315(uint32_t _id, MOTOR_INIT_t *_init) : Motor(_init, this), RS485(_id) {
    ID_Bind_Rx(RxMessage);
}

/**
 * @brief Motor_4315类的析构函数
 */
Motor_4315::~Motor_4315() = default;

/**
 * @brief 4315电机消息包获取任务
 */
void Motor_4315::RS485MessageGenerate() {
    uint32_t motorIndex = rs485_ID;

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
    nowAngle = (float)(RxMessage[7] | (RxMessage[8] << 8u) | (RxMessage[9] << 16u) | (RxMessage[10] << 24u) ) / 16384.0f * 360.0f;

    AngleCalc();
    if (stopFlag == 1) {
        motor4315_angle[rs485_ID] = (int32_t )(zeroAngle * 16384.0f / 360.0f);
    } else {
        motor4315_angle[rs485_ID] = (int32_t )(realAngle * 16384.0f / 360.0f);
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


/*4010电机类------------------------------------------------------------------*/
Motor_4010::Motor_4010(uint32_t id, MOTOR_INIT_t *motorInit) : CAN(id), Motor(motorInit, this) {
    ID_Bind_Rx(RxMessage);
    Stop();
}

void Motor_4010::SetTargetAngle(float _targetAngle) {
    stopFlag = false;
    targetAngle = _targetAngle;
}

void Motor_4010::SetTargetSpeed(float _targetSpeed) {
    stopFlag = false;
    targetSpeed = _targetSpeed;
}


void Motor_4010::CANMessageGenerate() {
    canQueue.push({ \
        ID, 0X08, \
        0xA1, 0x00, 0x00, 0x00, \
        static_cast<uint8_t>(motor4010_intensity), static_cast<uint8_t>(motor4010_intensity >> 8u), 0x00, 0x00 \
    });
}

void Motor_4010::Handle() {

    int16_t intensity;

    MotorStateUpdate();

    intensity = IntensityCalc();
    if (stopFlag) {
        motor4010_intensity = 0;
    } else {
        motor4010_intensity = intensity;
    }

    CANMessageGenerate();
}

void Motor_4010::MotorStateUpdate() {

    feedback.angle = RxMessage[6] | (RxMessage[7] << 8u);
    feedback.speed = (int16_t) (RxMessage[4] | (RxMessage[5] << 8u));
    feedback.moment = (int16_t) (RxMessage[2] | (RxMessage[3] << 8u));
    feedback.temp = (int8_t) RxMessage[1];

    switch (ctrlType) {
        case SPEED_Single: {
            state.speed = (float) (feedback.speed) / reductionRatio;
        }
        case POSITION_Double: {
            state.speed = (float) (feedback.speed) / reductionRatio;
            state.moment = feedback.moment;
            state.temperature = feedback.temp;
            state.angle = (float) (feedback.angle) * 360.0f / 16384.0f;

            thisAngle = feedback.angle;
            if (thisAngle <= lastRead) {
                if (lastRead - thisAngle > 8000)
                    realAngle += (thisAngle + 16384.0f - lastRead) * 360.0f / 16384.0f / reductionRatio;
                else
                    realAngle -= (lastRead - thisAngle) * 360.0f / 16384.0f / reductionRatio;
            } else {
                if (thisAngle - lastRead > 8000)
                    realAngle -= (lastRead + 16384.0f - thisAngle) * 360.0f / 16384.0f / reductionRatio;
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



int16_t Motor_4010::IntensityCalc() {
    int16_t intensity = 0;
    switch (ctrlType) {
        case DIRECT:
            intensity = (int16_t) targetAngle;
            break;

        case SPEED_Single:
            intensity = (int16_t) speedPID.PIDCalc(targetSpeed, state.speed);
            break;

        case POSITION_Double:
            float _targetSpeed = anglePID.PIDCalc(targetAngle, state.angle);
            intensity = (int16_t) speedPID.PIDCalc(_targetSpeed, state.speed);
            break;
    }
    return intensity;
}

Motor_4010::~Motor_4010() = default;
