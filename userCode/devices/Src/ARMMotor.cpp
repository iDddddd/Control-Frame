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
    if ((can2Queue.rear + 1) % MAX_MESSAGE_COUNT != can2Queue.front) {

        can2Queue.Data[can2Queue.rear].ID = can_ID;
        can2Queue.Data[can2Queue.rear].DLC = TxMessageDLC;
        can2Queue.Data[can2Queue.rear].canType = canType;
        can2Queue.Data[can2Queue.rear].message[0] = TxMessage[0];
        can2Queue.Data[can2Queue.rear].message[1] = TxMessage[1];
        can2Queue.Data[can2Queue.rear].message[2] = TxMessage[2];
        can2Queue.Data[can2Queue.rear].message[3] = TxMessage[3];
        can2Queue.Data[can2Queue.rear].message[4] = TxMessage[4];
        can2Queue.Data[can2Queue.rear].message[5] = TxMessage[5];
        can2Queue.Data[can2Queue.rear].message[6] = TxMessage[6];
        can2Queue.Data[can2Queue.rear].message[7] = TxMessage[7];

        can2Queue.rear = (can2Queue.rear + 1) % MAX_MESSAGE_COUNT;
    } else {
        can2Queue.rear = 0;
        can2Queue.front = 0;
    }
}

void SteppingMotor_v4::Handle() {
    if (RxMessage[1] == 0x9F) {
        ReachFlag = true;
        RxMessage[1] = 0x00;
    }
    if (SendFlag) {
        if (stopFlag) {
            TxMessageDLC = 0x05;
            TxMessage[0] = 0xF6;
            TxMessage[1] = 0x00;
            TxMessage[2] = 0x00;
            TxMessage[3] = 0x00;
            TxMessage[4] = 0x6B;
            CANMessageGenerate();
            SendFlag = false;
        } else {
            if (TarPos >= NowPos) {
                Direction = 0x04;
            } else {
                Direction = 0x14;
            }
            Pulse = (uint32_t) (abs(TarPos - NowPos) / 2.0f / PI * 200 * 16 * reductionRatio);
            NowPos = TarPos;
        }
        TxMessageDLC = 0x08;
        TxMessage[0] = 0xFD;
        TxMessage[1] = Direction;
        TxMessage[2] = 0xB0;
        TxMessage[3] = 0xFA;
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

void SteppingMotor_v4::Reset() {
    stopFlag = true;
    TarPos = 0;
    NowPos = 0;
}

void SteppingMotor_v4::Stop() {
    stopFlag = true;
    SendFlag = true;

}

SteppingMotor_v4::~SteppingMotor_v4() = default;


/*步进电机V5类------------------------------------------------------------------*/

SteppingMotor_v5::SteppingMotor_v5(COMMU_INIT_t *commuInit, MOTOR_INIT_t *motorInit) :
        Motor(motorInit, this), CAN(commuInit) {
    ID_Bind_Rx(RxMessage);

}

SteppingMotor_v5::~SteppingMotor_v5() = default;

void SteppingMotor_v5::CANMessageGenerate() {
    if ((can2Queue.rear + 1) % MAX_MESSAGE_COUNT != can2Queue.front) {

        can2Queue.Data[can2Queue.rear].ID = can_ID;
        can2Queue.Data[can2Queue.rear].DLC = TxMessageDLC;
        can2Queue.Data[can2Queue.rear].canType = canType;
        can2Queue.Data[can2Queue.rear].message[0] = TxMessage[0];
        can2Queue.Data[can2Queue.rear].message[1] = TxMessage[1];
        can2Queue.Data[can2Queue.rear].message[2] = TxMessage[2];
        can2Queue.Data[can2Queue.rear].message[3] = TxMessage[3];
        can2Queue.Data[can2Queue.rear].message[4] = TxMessage[4];
        can2Queue.Data[can2Queue.rear].message[5] = TxMessage[5];
        can2Queue.Data[can2Queue.rear].message[6] = TxMessage[6];
        can2Queue.Data[can2Queue.rear].message[7] = TxMessage[7];

        can2Queue.rear = (can2Queue.rear + 1) % MAX_MESSAGE_COUNT;
    } else {
        can2Queue.rear = 0;
        can2Queue.front = 0;
    }
}

void SteppingMotor_v5::Handle() {
    if (RxMessage[1] == 0x9F) {
        ReachFlag = true;
        RxMessage[1] = 0x00;
    }
    if (SendFlag) {
        if (stopFlag) {
            TxMessageDLC = 0x07;
            TxMessage[0] = 0xF6;
            TxMessage[1] = 0x00;
            TxMessage[2] = 0x00;
            TxMessage[3] = 0x00;
            TxMessage[4] = 0x00;
            TxMessage[5] = 0x00;
            TxMessage[6] = 0x6B;
            CANMessageGenerate();
            SendFlag = false;
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
            TxMessage[2] = 0x0F;
            TxMessage[3] = 0xA0;
            TxMessage[4] = 0xF0;
            TxMessage[5] = Pulse >> 24u;
            TxMessage[6] = Pulse >> 16u;
            TxMessage[7] = Pulse >> 8u;
            CANMessageGenerate();
            can_ID += 1;
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
            can_ID -= 1;
        }
        SendFlag = false;
    }
}

void SteppingMotor_v5::SetTargetPosition(float tarpos) {
    stopFlag = false;
    SendFlag = true;
    TarPos = tarpos;
}

void SteppingMotor_v5::Reset() {
    stopFlag = true;
    TxMessageDLC = 0x03;
    TxMessage[0] = 0x0A;
    TxMessage[1] = 0x6D;
    TxMessage[2] = 0x6B;
    TxMessage[3] = 0x00;
    TxMessage[4] = 0x00;
    TxMessage[5] = 0x00;
    TxMessage[6] = 0x00;
    TxMessage[7] = 0x00;
    CANMessageGenerate();
}

void SteppingMotor_v5::Stop() {
    stopFlag = true;
    SendFlag = true;

}


