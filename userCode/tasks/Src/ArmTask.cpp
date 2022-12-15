//
// Created by mac on 2022/12/14.
//

#include "ArmTask.h"


MOTOR_TYPE_e arm1Type = ARM1;
MOTOR_TYPE_e arm2Type = ARM2;
MOTOR_TYPE_e arm3Type = ARM3;

ARMMotor ARMMotor1(&arm1Type);
ARMMotor ARMMotor2(&arm2Type);
ARMMotor ARMMotor3(&arm3Type);

bool ArmStopFlag = true;

void ARMHandle(){
    ARMMotor1.Handle();
    ARMMotor2.Handle();
    ARMMotor3.Handle();
}
