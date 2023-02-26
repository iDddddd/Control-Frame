//
// Created by mac on 2022/12/14.
//

#include "ArmTask.h"


PID_Regulator_t pidRegulator3 = {//此为储存pid参数的结构体，四个底盘电机共用
        .kp = 2.0f,
        .ki = 0.0f,
        .kd = 0.03f,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000
};
PID_Regulator_t pidRegulator4 = {//此为储存pid参数的结构体，四个底盘电机共用
        .kp = 0.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000 //4010电机输出电流上限，可以调小，勿调大
};
PID_Regulator_t pidRegulator5 = {//此为储存pid参数的结构体，四个底盘电机共用
        .kp = 20.0f,
        .ki = 0.2f,
        .kd = 0.2f,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000
};
PID_Regulator_t pidRegulator6 = {//此为储存pid参数的结构体，四个底盘电机共用
        .kp = 3.0f,
        .ki = 0.0f,
        .kd = 0.0f,
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
COMMU_INIT_t arm2CommuInit = {
        ._id = 0x146,
        .ctrlType = POSITION_Double,
        .canType = can2

};
MOTOR_INIT_t arm2MotorInit = {
        .speedPIDp = &pidRegulator5,
        .anglePIDp = &pidRegulator6,
        .reductionRatio = 1.0f

};
COMMU_INIT_t arm3CommuInit = {
        ._id = 0x01,
        .ctrlType = DIRECT,
        .canType = can2

};
MOTOR_INIT_t arm3MotorInit = {
        .speedPIDp = nullptr,
        .anglePIDp = nullptr,
        .reductionRatio = 1.0f

};
//Motor_4010 TrayMotor(&trayCommuInit, &trayMotorInit);
//Motor_4310 ArmMotor1(&arm1CommuInit, &arm1MotorInit);
//Motor_4010 ArmMotor2(&arm2CommuInit, &arm2MotorInit);
//Emm42Motor ArmMotorZ(&arm3CommuInit,&arm3MotorInit);
bool ArmStopFlag = true;
float Position,Angle;

void ArmStop() {
    ArmStopFlag = true;
   // TrayMotor.Stop();
    //ArmMotor1.Stop();
   // ArmMotor2.Stop();
   // ArmMotorZ.Stop();
}

void ArmAngleCalc() {
    // TrayMotor.SetTargetAngle(Angle);
     //ArmMotor1.SetTargetAngle(Position);
    // ArmMotor2.SetTargetAngle(Angle);
   // ArmMotorZ.SetTargetPosition(Position);
}

void ArmSetAngle(float position, float angle) {
    ArmStopFlag = false;
    Position = position;
    Angle = angle;

}

void ARMHandle() {
    if (!ArmStopFlag) {
        ArmAngleCalc();
    }
}




