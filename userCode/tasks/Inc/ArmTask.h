//
// Created by mac on 2022/12/14.
//

#ifndef RM_FRAME_C_ARMTASK_H
#define RM_FRAME_C_ARMTASK_H


#include "Device.h"
#include "Motor.h"
#include "OtherMotor.h"
#include "Servo.h"

#define PI 3.1415926f
void AngleCalc();

void ArmStop();

void ArmSetAngle(float Arm1Angle, float Arm2Angle);

void ArmAngleCalc();

#endif //RM_FRAME_C_ARMTASK_H
