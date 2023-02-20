//
// Created by mac on 2022/12/14.
//

#include "OtherMotor.h"

/*静态成员变量声明------------------------------------------------------------------*/

int16_t Motor_4010::Intensity;

/*4010电机类------------------------------------------------------------------*/

Motor_4010::Motor_4010(COMMU_INIT_t *commuInit, MOTOR_INIT_t *motorInit) : CAN(commuInit), Motor(motorInit, this) {
    id = commuInit->_id - 0x145; //0x141-0x144被FOUR_Motor4010占用
    ID_Bind_Rx(RxMessage);

}

void Motor_4010::SetTargetAngle(float _targetAngle) {
    stopFlag = false;
    targetAngle = _targetAngle;
}


void Motor_4010::CANMessageGenerate() {

    if ((canQueue.rear + 1) % canQueue.MAX_MESSAGE_COUNT != canQueue.front) {

        canQueue.Data[canQueue.rear].ID = can_ID;
        canQueue.Data[canQueue.rear].canType = canType;
        canQueue.Data[canQueue.rear].message[0] = 0xA1;
        canQueue.Data[canQueue.rear].message[1] = 0;
        canQueue.Data[canQueue.rear].message[2] = 0;
        canQueue.Data[canQueue.rear].message[3] = 0;
        canQueue.Data[canQueue.rear].message[4] = motor4010_intensity[id];
        canQueue.Data[canQueue.rear].message[5] = motor4010_intensity[id] >> 8u;
        canQueue.Data[canQueue.rear].message[6] = 0;
        canQueue.Data[canQueue.rear].message[7] = 0;

        canQueue.rear = (canQueue.rear + 1) % canQueue.MAX_MESSAGE_COUNT;
    }

}

void Motor_4010::Handle() {
    int16_t intensity[8];

    MotorStateUpdate();
    intensity[id] = IntensityCalc();
    Intensity = intensity[id];
    if (stopFlag) {
        motor4010_intensity[id] = 0;
    } else {
        motor4010_intensity[id] = intensity[id];
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
            // intensity = speedPID.PIDCalc(targetSpeed, state.speed);
            break;

        case POSITION_Double:
            float _targetSpeed = anglePID.PIDCalc(targetAngle, state.angle);
            intensity = (int16_t) speedPID.PIDCalc(_targetSpeed, state.speed);
            break;
    }
    return intensity;
}

Motor_4010::~Motor_4010() = default;

/*4310电机类------------------------------------------------------------------*/
Motor_4310::Motor_4310(COMMU_INIT_t *commuInit, MOTOR_INIT_t *motorInit) : CAN(commuInit), Motor(motorInit, this) {

}

Motor_4310::~Motor_4310() = default;

void Motor_4310::Init() {
    CAN_TxHeaderTypeDef txHeaderTypeDef;
    uint32_t box;
    uint8_t InitMessage[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

    txHeaderTypeDef.StdId = 0x101;
    txHeaderTypeDef.DLC = 0x08;
    txHeaderTypeDef.IDE = CAN_ID_STD;
    txHeaderTypeDef.RTR = CAN_RTR_DATA;
    txHeaderTypeDef.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(&hcan2, &txHeaderTypeDef, InitMessage, &box);
}

void Motor_4310::SetTargetAngle(float _targetAngle) {
    stopFlag = false;
    targetAngle = _targetAngle;
}

void Motor_4310::SetTargetSpeed(float _targetSpeed) {
    stopFlag = false;
    targetSpeed = _targetSpeed;
}

void Motor_4310::CANMessageGenerate() {
    if ((canQueue.rear + 1) % canQueue.MAX_MESSAGE_COUNT != canQueue.front) {

        canQueue.Data[canQueue.rear].ID = can_ID;
        canQueue.Data[canQueue.rear].canType = canType;
        canQueue.Data[canQueue.rear].message[3] = Motor4310_Angle;
        canQueue.Data[canQueue.rear].message[2] = Motor4310_Angle >> 8u;
        canQueue.Data[canQueue.rear].message[1] = Motor4310_Angle >> 16u;
        canQueue.Data[canQueue.rear].message[0] = Motor4310_Angle >> 24u;
        canQueue.Data[canQueue.rear].message[4] = 0x00;
        canQueue.Data[canQueue.rear].message[5] = 0x00;
        canQueue.Data[canQueue.rear].message[6] = 0x00;
        canQueue.Data[canQueue.rear].message[7] = 0x3F;

        canQueue.rear = (canQueue.rear + 1) % canQueue.MAX_MESSAGE_COUNT;
    }
}

void Motor_4310::Handle() {
    uint32_t Angle = AngleCalc();
    if (stopFlag) {
        Motor4310_Angle = 0;
    } else {
        Motor4310_Angle = Angle;
    }
    CANMessageGenerate();

}

int32_t Motor_4310::AngleCalc() {
    float Angle;
    Angle = targetAngle * 12.5f / 360;
    return float_to_uint(Angle, -12.5, 12.5, 16);
}

float Motor_4310::uint_to_float(int x_int, float x_min, float x_max, int bits) {
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
}

int Motor_4310::float_to_uint(float x, float x_min, float x_max, int bits) {
/// Converts a float to an unsigned int, given range and number of bits///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x - offset) * ((float) ((1 << bits) - 1)) / span);
}

/*Emm42电机类------------------------------------------------------------------*/
Emm42Motor::Emm42Motor(COMMU_INIT_t *commuInit, MOTOR_INIT_t *motorInit) : Motor(motorInit, this),
                                                                                         CAN(commuInit) {
    ID_Bind_Rx(RxMessage);
}

void Emm42Motor::GeneratePositon() {
    CAN_TxHeaderTypeDef txHeaderTypeDef;
    uint32_t box;
    uint8_t Message[2] = {0x36, 0x6B};

    txHeaderTypeDef.StdId = 0x01;
    txHeaderTypeDef.DLC = 0x02;
    txHeaderTypeDef.IDE = CAN_ID_STD;
    txHeaderTypeDef.RTR = CAN_RTR_DATA;
    txHeaderTypeDef.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(&hcan2, &txHeaderTypeDef, Message, &box);
}

void Emm42Motor::CANMessageGenerate() {
    if ((canQueue.rear + 1) % canQueue.MAX_MESSAGE_COUNT != canQueue.front) {

        canQueue.Data[canQueue.rear].ID = can_ID;
        canQueue.Data[canQueue.rear].canType = canType;
        canQueue.Data[canQueue.rear].message[0] = 0xFD;
        canQueue.Data[canQueue.rear].message[1] = Emm42Motor_Dir;
        canQueue.Data[canQueue.rear].message[2] = 0xFF;
        canQueue.Data[canQueue.rear].message[3] = 0x00;
        canQueue.Data[canQueue.rear].message[4] = Emm42Motor_Pos >> 16u;
        canQueue.Data[canQueue.rear].message[5] = Emm42Motor_Pos >> 8u;
        canQueue.Data[canQueue.rear].message[6] = Emm42Motor_Pos;
        canQueue.Data[canQueue.rear].message[7] = 0x6B;

        canQueue.rear = (canQueue.rear + 1) % canQueue.MAX_MESSAGE_COUNT;
    }
}

void Emm42Motor::Handle() {
    GeneratePositon();
    PositionCalc();
    int32_t targetPos = targetPosition / 8.4f * 65536.0f;
    int32_t Pos = targetPos - NowPos;
    if (Pos <= 0) {
        Emm42Motor_Dir = 0x02;//0表示往下，2为速度值，其后还有0xFF
        Pos *= -1;
    } else {
        Emm42Motor_Dir = 0x12;
    }
    if (stopFlag) {
        Emm42Motor_Pos = 0;
    } else {
        Emm42Motor_Pos = Pos / 65536 * 3200 ;
    }
    CANMessageGenerate();
}

void Emm42Motor::PositionCalc() {

        NowPos = (RxMessage[1] << 24u) | (RxMessage[2] << 16u) | (RxMessage[3] << 8u) | (RxMessage[4]);

}

void Emm42Motor::SetTargetPosition(float _targetposition) {
    stopFlag = false;
    targetPosition = _targetposition;
}


Emm42Motor::~Emm42Motor() = default;


