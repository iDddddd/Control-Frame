//
// Created by mac on 2022/12/14.
//

#ifndef RM_FRAME_C_OTHERMOTOR_H
#define RM_FRAME_C_OTHERMOTOR_H

#include "Device.h"
#include "can.h"
#include "CatchControl.h"

typedef enum {
    ARM1 = 0,
    ARM2,
} MOTOR_TYPE_e;


class ARMMotor : private Device {
public:

    static uint8_t arm1_Initmessage[2];
    static uint8_t arm2_Initmessage[8];

    static uint8_t arm1message[8];
    static uint8_t arm2message[8];


    static float feedback_moment[3];

    static void Init();

    static void ARM1_Init();

    static void ARM2_Init();

    //机械臂4310发送
    static void ARMCAN1PackageSend();

    //机械臂4010发送
    static void ARMCAN2PackageSend();

    static void PackageSend();

    static void IT_Handle(CAN_HandleTypeDef *hcan);

    bool stopFlag{false};

    MOTOR_TYPE_e motorType;
    float targetSpeed = 0;
    float targetAngle = 0;

    ARMMotor(MOTOR_TYPE_e *MotorType);

    ~ARMMotor();

    void Handle() override;

    void ErrorHandle() override;

    void Stop();

private:

    void ARMStop();

    void ARMCAN1MessageGenerate();

    void ARMCAN2MessageGenerate();

};


class TRAYMotor : private Device {
public:
    static uint8_t traymessage[3][8];
    static uint8_t trayflag;

    //底盘旋转4010发送
    static void TrayPackageSend();

    TRAYMotor();

    ~TRAYMotor();

    void Handle() override;

    void ErrorHandle() override;

private:
    void MotorStateUpdate();

    void TRAYFlagGenerate();

};

#endif //RM_FRAME_C_OTHERMOTOR_H
