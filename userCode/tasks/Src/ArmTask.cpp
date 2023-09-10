//
// Created by mac on 2022/12/14.
//

#include "ArmTask.h"


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
SteppingMotor_v4 Joint4Motor(&Joint4CommuInit, &Joint4MotorInit);
SteppingMotor_v4 Joint5Motor(&Joint5CommuInit, &Joint5MotorInit);
StepperMotor ClawMotor(&ClawMotorInit);

bool ArmStopFlag = true;
bool ArmMoveFlag = false;
static float joint1Angle, joint2Angle, joint3Angle, joint4Angle, joint5Angle;

void ArmStop() {
    ArmStopFlag = true;
    Joint1Motor.Stop();
    Joint2Motor.Stop();
    Joint3Motor.Stop();
    Joint4Motor.Stop();
    Joint5Motor.Stop();
    ClawMotor.Stop();
}

void ArmSet(float Joint1Pos, float Joint2Pos, float Joint3Pos, float Joint4Pos, float Joint5Pos) {
    ArmStopFlag = false;
    Joint1Motor.SetTargetPosition(Joint1Pos);
    Joint2Motor.SetTargetPosition(Joint2Pos);
    Joint3Motor.SetTargetPosition(Joint3Pos);
    Joint4Motor.SetTargetPosition(Joint4Pos);
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

void ARMHandle() {
    if (!ArmStopFlag) {
        if (ArmMoveFlag) {
            Joint1Motor.MoveTo();
            Joint2Motor.MoveTo();
            Joint3Motor.MoveTo();
            Joint4Motor.MoveTo();
            Joint5Motor.MoveTo();
            ArmMoveFlag = false;
        }
    }
}




