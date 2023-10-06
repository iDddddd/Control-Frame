//
// Created by mac on 2022/12/14.
//

#ifndef RM_FRAME_C_ARMTASK_H
#define RM_FRAME_C_ARMTASK_H


#include "Device.h"
#include "Motor.h"
#include "ArmMotor.h"
//#include "Servo.h"
#include "StepperMotor.h"

#define l2 152.8f
#define l3 130.46f
#define l4 110.69

class ArmTask {
public:
    static void ArmStop();
    static void ArmCalc(float x,float y,float z);


    static float Angle[4];

};

#endif //RM_FRAME_C_ARMTASK_H
