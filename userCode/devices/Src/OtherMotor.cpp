//
// Created by mac on 2022/12/14.
//

#include "OtherMotor.h"

/*静态成员变量声明------------------------------------------------------------------*/
uint8_t Motor_4310::InitMessage[2] = {0x30, 0x6B};

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
      //  canQueue.Data[canQueue.rear].Serial = canSerial;
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

    txHeaderTypeDef.StdId = 0x01;
    txHeaderTypeDef.DLC = 0x08;
    txHeaderTypeDef.IDE = CAN_ID_STD;
    txHeaderTypeDef.RTR = CAN_RTR_DATA;
    txHeaderTypeDef.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(&hcan2, &txHeaderTypeDef, InitMessage, 0);
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

}

void Motor_4310::Handle() {
    uint32_t id = 0;

    if (stopFlag) {
        motor4310_intensity[id] = 0;
    } else {
        motor4310_intensity[id] = 1;//
    }

    CANMessageGenerate();
}




