//
// Created by mac on 2022/12/14.
//

#include "OtherMotor.h"

/*静态成员变量声明------------------------------------------------------------------*/


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
    if (stopFlag) {
        motor4010_intensity[id] = 0;
    } else {
        motor4010_intensity[id] = intensity[id];
    }

    CANMessageGenerate();
}

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

int16_t Motor_4010::IntensityCalc() {
    int16_t intensity = 0;
    switch (ctrlType) {
        case DIRECT:
            intensity = targetAngle;
            break;

        case SPEED_Single:
            // intensity = speedPID.PIDCalc(targetSpeed, state.speed);
            break;

        case POSITION_Double:
            float _targetSpeed = anglePID.PIDCalc(targetAngle, state.angle);
            intensity = speedPID.PIDCalc(_targetSpeed, state.speed);
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
        canQueue.Data[canQueue.rear].message[0] = Motor4310_Angle;
        canQueue.Data[canQueue.rear].message[1] = Motor4310_Angle >> 8u;
        canQueue.Data[canQueue.rear].message[2] = Motor4310_Angle >> 16u;
        canQueue.Data[canQueue.rear].message[3] = Motor4310_Angle >> 24u;
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
