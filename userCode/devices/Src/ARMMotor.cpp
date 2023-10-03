//
// Created by mac on 2022/12/14.
//

#include "ArmMotor.h"

/*静态成员变量声明------------------------------------------------------------------*/



/*步进电机V4类------------------------------------------------------------------*/
SteppingMotor_v4::SteppingMotor_v4(uint32_t id, MOTOR_INIT_t *motorInit) :
    Motor(motorInit, this), CAN(id) {
    ID_Bind_Rx(RxMessage);
}


void SteppingMotor_v4::CANMessageGenerate() {
    if ((canQueue.rear + 1) % MAX_MESSAGE_COUNT != canQueue.front) {

        canQueue.Data[canQueue.rear].ID = ID;
        canQueue.Data[canQueue.rear].DLC = TxMessageDLC;
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
            if (TarPos >= NowPos) {
                Direction = 0x01;
            } else {
                Direction = 0x11;
            }
            Pulse = (uint32_t) (abs(TarPos - NowPos) / 2.0f / PI * 200 * 16 * reductionRatio);
            NowPos = TarPos;
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
        SendFlag = false;
    }
}

void SteppingMotor_v4::SetTargetPosition(float pos) {
    stopFlag = false;
    SendFlag = true;
    TarPos = pos;
}

SteppingMotor_v4::~SteppingMotor_v4() = default;


/*步进电机V5类------------------------------------------------------------------*/

SteppingMotor_v5::SteppingMotor_v5(uint32_t id, MOTOR_INIT_t *motorInit) :
        Motor(motorInit, this), CAN(id) {
    ID_Bind_Rx(RxMessage);
}

SteppingMotor_v5::~SteppingMotor_v5() = default;

void SteppingMotor_v5::CANMessageGenerate() {;
    if ((canQueue.rear + 1) % MAX_MESSAGE_COUNT != canQueue.front) {

        canQueue.Data[canQueue.rear].ID = ID;
        canQueue.Data[canQueue.rear].DLC = TxMessageDLC;
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
            ID += 1;
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
            ID -= 1;
        }
        SendFlag = false;
    }
}

void SteppingMotor_v5::SetTargetPosition(float tarpos) {
    stopFlag = false;
    SendFlag = true;
    TarPos = tarpos;
}


