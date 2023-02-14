//
// Created by mac on 2022/12/14.
//

#include "ArmTask.h"


PID_Regulator_t pidRegulator3 = {//此为储存pid参数的结构体，四个底盘电机共用
        .kp = 2.0f,
        .ki = 0.002f,
        .kd = 0.0001f,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000
};
PID_Regulator_t pidRegulator4 = {//此为储存pid参数的结构体，四个底盘电机共用
        .kp = 0.5f,
        .ki = 0.001f,
        .kd = 0.00001f,
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
        .canType = can2

};
MOTOR_INIT_t arm1MotorInit = {
        .speedPIDp = nullptr,
        .anglePIDp = nullptr,
        .reductionRatio = 1.0f
};
COMMU_INIT_t arm1CommuInit = {
        ._id = 0x101,
        .ctrlType = DIRECT,
        .canType = can2

};
Motor_4010 TrayMotor(&trayCommuInit, &trayMotorInit);
Motor_4310 ArmMotor1(&arm1CommuInit, &arm1MotorInit);
bool ArmStopFlag = true;
float Angle;

void ArmStop() {
    ArmStopFlag = true;
    TrayMotor.Stop();
    ArmMotor1.Stop();
}

void ArmAngleCalc(){
    TrayMotor.SetTargetAngle(Angle);
    ArmMotor1.SetTargetAngle(Angle);
}
void ArmSetAngle(float angle){
    ArmStopFlag = false;
    Angle = angle;

}


void ARMHandle() {
    if (!ArmStopFlag) {
        ArmAngleCalc();
    }
}




