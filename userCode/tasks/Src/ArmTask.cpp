//
// Created by mac on 2022/12/14.
//

#include "ArmTask.h"


PID_Regulator_t pidRegulator3 = {//此为储存pid参数的结构体，四个底盘电机共用
        .kp = 1.0f,
        .ki = 0.0f,
        .kd = 0,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000
};
PID_Regulator_t pidRegulator4 = {//此为储存pid参数的结构体，四个底盘电机共用
        .kp = 0.1f,
        .ki = 0.0f,
        .kd = 0,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000 //4010电机输出电流上限，可以调小，勿调大
};
MOTOR_INIT_t trayMotorInit = {
        .speedPIDp = &pidRegulator3,
        .anglePIDp = &pidRegulator4,
        .reductionRatio = 1.0f

};

COMMU_INIT_t trayCommuInit = {
        ._id = 0x145,
        .ctrlType = POSITION_Double,
   //     .Serial = TWO

};

Motor_4010 TrayMotor(&trayCommuInit, &trayMotorInit);

bool ArmStopFlag = false;


void ArmStop() {
    ArmStopFlag = true;
}

void ArmAngleCalc(){
    TrayMotor.SetTargetAngle(300);
}

void ARMHandle() {
    if (!ArmStopFlag) {
        ArmAngleCalc();
    }
}




