//
// Created by mac on 2022/12/14.
//

#include "ArmMotor.h"

/*静态成员变量声明------------------------------------------------------------------*/



/*Emm42电机类------------------------------------------------------------------*/
SteppingMotor::SteppingMotor(COMMU_INIT_t *commuInit, MOTOR_INIT_t *motorInit) :
        Motor(motorInit, this), CAN(commuInit) {
}


void SteppingMotor::CANMessageGenerate() {
    if ((canQueue.rear + 1) % MAX_MESSAGE_COUNT != canQueue.front) {

        canQueue.Data[canQueue.rear].ID = can_ID;
        canQueue.Data[canQueue.rear].canType = canType;
        canQueue.Data[canQueue.rear].message[0] = TxMessage[0];
        canQueue.Data[canQueue.rear].message[1] = TxMessage[1];
        canQueue.Data[canQueue.rear].message[2] = TxMessage[2];
        canQueue.Data[canQueue.rear].message[3] = TxMessage[3];
        canQueue.Data[canQueue.rear].message[4] = TxMessage[4];
        canQueue.Data[canQueue.rear].message[5] = TxMessage[5];
        canQueue.Data[canQueue.rear].message[6] = TxMessage[6];
        canQueue.Data[canQueue.rear].message[7] = TxMessage[7];

        canQueue.rear = (canQueue.rear + 1) % MAX_MESSAGE_COUNT;
    } else {
        canQueue.rear = 0;
        canQueue.front = 0;
    }
}

void SteppingMotor::Handle() {
    TxMessage[0] = 0x36;
    TxMessage[0] = 0x6B;

    NowPos = (float ) (RxMessage[1] << 24u | RxMessage[2] << 16u | RxMessage[3] << 8u | RxMessage[4]) * 360.0f /
             65536.0f / reductionRatio;

    CANMessageGenerate();
}

void SteppingMotor::MoveTo() {

    if (stopFlag) {
        Pulse = 0;

    } else {
        if (TarPos >= Position) {
            Direction = 0x11;
        } else {
            Direction = 0x01;
        }
        Pulse = (uint32_t) (abs(TarPos - Position) / 2.0f / PI * 200 * 16 * reductionRatio);
    }
    TxMessage[0] = 0xFD;
    TxMessage[1] = Direction;
    TxMessage[2] = 0xFF;
    TxMessage[3] = 0x00;
    TxMessage[4] = Pulse >> 16u;
    TxMessage[5] = Pulse >> 8u;
    TxMessage[6] = Pulse;
    TxMessage[7] = 0x6B;
    CANMessageGenerate();
}

void SteppingMotor::SetTargetPosition(float pos) {
    stopFlag = false;
    TarPos = pos;
}

SteppingMotor::~SteppingMotor() = default;
