//
// Created by mac on 2022/12/14.
//

#include "ArmTask.h"

#include <cmath>

float ArmTask::Angle[4]{0};

MOTOR_INIT_t Joint1MotorInit = {
        .speedPIDp = nullptr,
        .anglePIDp = nullptr,
        .ctrlType = DIRECT,
        .reductionRatio = 60.0f
};
MOTOR_INIT_t Joint2MotorInit = {
        .speedPIDp = nullptr,
        .anglePIDp = nullptr,
        .ctrlType = DIRECT,
        .reductionRatio = 30.0f
};
MOTOR_INIT_t Joint3MotorInit = {
        .speedPIDp = nullptr,
        .anglePIDp = nullptr,
        .ctrlType = DIRECT,
        .reductionRatio = 30.0f
};
MOTOR_INIT_t Joint4MotorInit = {
        .speedPIDp = nullptr,
        .anglePIDp = nullptr,
        .ctrlType = DIRECT,
        .reductionRatio = 20.0f
};
MOTOR_INIT_t Joint5MotorInit = {
        .speedPIDp = nullptr,
        .anglePIDp = nullptr,
        .ctrlType = DIRECT,
        .reductionRatio = 20.0f
};
MOTOR_INIT_t ClawMotorInit = {
        .speedPIDp = nullptr,
        .anglePIDp = nullptr,
        .ctrlType = DIRECT,
        .reductionRatio = 20.0f
};


COMMU_INIT_t Joint1CommuInit = {
        ._id = 0x0100,
        .canType = can2

};
COMMU_INIT_t Joint2CommuInit = {
        ._id = 0x0200,
        .canType = can2

};
COMMU_INIT_t Joint3CommuInit = {
        ._id = 0x0300,
        .canType = can2

};
COMMU_INIT_t Joint4CommuInit = {
        ._id = 0x04,
        .canType = can2

};
COMMU_INIT_t Joint5CommuInit = {
        ._id = 0x05,
        .canType = can2

};
SteppingMotor_v5 Joint1Motor(&Joint1CommuInit, &Joint1MotorInit);
SteppingMotor_v5 Joint2Motor(&Joint2CommuInit, &Joint2MotorInit);
SteppingMotor_v5 Joint3Motor(&Joint3CommuInit, &Joint3MotorInit);
//SteppingMotor_v4 Joint4Motor(&Joint4CommuInit, &Joint4MotorInit);
SteppingMotor_v4 Joint5Motor(&Joint5CommuInit, &Joint5MotorInit);
StepperMotor ClawMotor(&ClawMotorInit);

bool ArmStopFlag = true;
bool ArmMoveFlag = false;
static float joint1Angle, joint2Angle, joint3Angle, joint4Angle, joint5Angle;

void ArmTask::ArmStop() {
    ArmStopFlag = true;
    Joint1Motor.Stop();
    Joint2Motor.Stop();
    Joint3Motor.Stop();
   // Joint4Motor.Stop();
    Joint5Motor.Stop();
    ClawMotor.Stop();
}

void ArmJointSet(float Joint1Pos, float Joint2Pos, float Joint3Pos, float Joint4Pos, float Joint5Pos) {
    ArmStopFlag = false;
    Joint1Motor.SetTargetPosition(Joint1Pos);
    Joint2Motor.SetTargetPosition(Joint2Pos);
    Joint3Motor.SetTargetPosition(Joint3Pos);
   // Joint4Motor.SetTargetPosition(Joint4Pos);
    Joint5Motor.SetTargetPosition(Joint5Pos);
    ArmMoveFlag = true;

}

void ClawSet(uint8_t clawflag) {
    if (clawflag == 1) {
        ClawMotor.Grab(true);
    } else if (clawflag == 0) {
        ClawMotor.Grab(false);
    }
}

void ArmPositionSet(float x, float y, float z){
    ArmTask::ArmCalc(x,y,z);
    ArmJointSet(ArmTask::Angle[0], ArmTask::Angle[1], ArmTask::Angle[2], 0, ArmTask::Angle[3]);
}
void ArmTask::ArmCalc(float x,float y,float z){
    float d1 = sqrtf(x*x+y*y);
    float d2 = sqrtf(d1*d1+(z+l4)*(z+l4));
    float angle1 = acosf((l3*l3-l2*l2-d2*d2)/(-2*l2*d2));
    float angle2 = acosf((d2*d2-l2*l2-l3*l3)/(-2*l2*l3));
    float angle3 = PI - angle1 - angle2;
    float angle4 = atan2f(z+l4,d1);
    float angle5 = PI/2 - angle4;

    Angle[0] = atan2f(x,y);
    Angle[1] = PI/2 - angle1 - angle4;
    Angle[2] = PI/2 - angle2;
    Angle[3] = PI/2 - angle3 - angle5;
    for(float & i : Angle) {
        if(i != i){//判断是否为nan
            i = 0;
        }
    }
}

void ArmReset() {
    Joint1Motor.Reset();
    Joint2Motor.Reset();
    Joint3Motor.Reset();
   // Joint4Motor.Reset();
    Joint5Motor.Reset();

}



