//
// Created by mac on 2022/12/14.
//

#include "ArmMotor.h"

/*静态成员变量声明------------------------------------------------------------------*/



/*步进电机V4类------------------------------------------------------------------*/
SteppingMotor_v4::SteppingMotor_v4(COMMU_INIT_t *commuInit, MOTOR_INIT_t *motorInit) :
        Motor(motorInit, this), CAN(commuInit) {
    ID_Bind_Rx(RxMessage);
}


void SteppingMotor_v4::CANMessageGenerate() {
    if ((canQueue.rear + 1) % MAX_MESSAGE_COUNT != canQueue.front) {

        canQueue.Data[canQueue.rear].ID = can_ID;
        canQueue.Data[canQueue.rear].DLC = TxMessageDLC;
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

void SteppingMotor_v4::Handle() {
    if(SendFlag) {
        if (stopFlag) {
            Pulse = 0;
        } else {
            if (TarPos >= Position) {
                Direction = 0x11;
            } else {
                Direction = 0x01;
            }
            Pulse = (uint32_t) (abs(TarPos - Position) / 2.0f / PI * 200 * 16 * reductionRatio);
            Position = TarPos;
        }
        TxMessageDLC = 0x08;
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
}

void SteppingMotor_v4::SetTargetPosition(float pos) {
    stopFlag = false;
    SendFlag = true;
    TarPos = pos;
}

SteppingMotor_v4::~SteppingMotor_v4() = default;


/*步进电机V5类------------------------------------------------------------------*/

SteppingMotor_v5::SteppingMotor_v5(COMMU_INIT_t *commuInit, MOTOR_INIT_t *motorInit) :
        Motor(motorInit, this), CAN(commuInit) {
    ID_Bind_Rx(RxMessage);
}

SteppingMotor_v5::~SteppingMotor_v5() = default;

void SteppingMotor_v5::CANMessageGenerate() {;
    if ((canQueue.rear + 1) % MAX_MESSAGE_COUNT != canQueue.front) {

        canQueue.Data[canQueue.rear].ID = can_ID;
        canQueue.Data[canQueue.rear].DLC = TxMessageDLC;
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

void SteppingMotor_v5::Handle() {
/*
    TxMessage[0] = 0x36;
    TxMessage[1] = 0x6B;
    TxMessageDLC = 0x02;

    nowPos = (int32_t) (RxMessage[2] << 24u | RxMessage[3] << 16u | RxMessage[4] << 8u | RxMessage[5]);
    if (RxMessage[1] == 0x01) {
        nowPos = -nowPos;
    }
    NowPos = ((float) nowPos * 2.0f * PI) / 65536.0f / reductionRatio;
*/
    if (SendFlag) {
        if (stopFlag) {
            Pulse = 0;
        } else {

            if (TarPos >= 0) {
                Direction = 0x00;
            } else {
                Direction = 0x01;
            }
            Pulse = (uint32_t) (abs(TarPos) / 2.0f / PI * 200.0f * 16.0f * reductionRatio);
            TxMessageDLC = 0x08;
            TxMessage[0] = 0xFD;
            TxMessage[1] = Direction;
            TxMessage[2] = 0x05;
            TxMessage[3] = 0xDC;
            TxMessage[4] = 0xC8;
            TxMessage[5] = Pulse >> 24u;
            TxMessage[6] = Pulse >> 16u;
            TxMessage[7] = Pulse >> 8u;
            CANMessageGenerate();
            TxMessageDLC = 0x05;
            TxMessage[0] = 0xFD;
            TxMessage[1] = Pulse;
            TxMessage[2] = 0x01;
            TxMessage[3] = 0x00;
            TxMessage[4] = 0x6B;
            TxMessage[5] = 0x00;
            TxMessage[6] = 0x00;
            TxMessage[7] = 0x00;
            CANMessageGenerate();
            SendFlag = false;
        }
    }
}

void SteppingMotor_v5::SetTargetPosition(float tarpos) {
    stopFlag = false;
    SendFlag = true;
    TarPos = tarpos;
}


