//
// Created by mac on 2022/12/14.
//

#include "ArmTask.h"


PID_Regulator_t pidRegulator5 = {//此为储存pid参数的结构体，四个底盘电机共用
         .kp = 0.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000
};
PID_Regulator_t pidRegulator6 = {//此为储存pid参数的结构体，四个底盘电机共用
       .kp = 0.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000 //4010电机输出电流上限，可以调小，勿调大
};
MOTOR_INIT_t trayMotorInit = {
        .speedPIDp = nullptr,
        .anglePIDp = nullptr,
        .ctrlType = DIRECT,
        .reductionRatio = 1.0f

};

COMMU_INIT_t trayCommuInit = {
        ._id = 0x145,
        .canType = can2

};
MOTOR_INIT_t arm1MotorInit = {
        .speedPIDp = nullptr,
        .anglePIDp = nullptr,
        .ctrlType = DIRECT,
        .reductionRatio = 1.0f
};
COMMU_INIT_t arm1CommuInit = {
        ._id = 0x101,
        .canType = can2

};
COMMU_INIT_t arm2CommuInit = {
        ._id = 0x146,
        .canType = can2

};
MOTOR_INIT_t arm2MotorInit = {
        .speedPIDp = &pidRegulator5,
        .anglePIDp = &pidRegulator6,
        .ctrlType = DIRECT,
        .reductionRatio = 1.0f

};
COMMU_INIT_t arm3CommuInit = {
        ._id = 0x01,
        .canType = can2

};
MOTOR_INIT_t arm3MotorInit = {
        .speedPIDp = nullptr,
        .anglePIDp = nullptr,
        .ctrlType = DIRECT,
        .reductionRatio = 1.0f

};
//Motor_4010_TRAY TrayMotor(&trayCommuInit, &trayMotorInit);
Motor_4310 ArmMotor1(&arm1CommuInit, &arm1MotorInit);
Motor_4010 ArmMotor2(&arm2CommuInit, &arm2MotorInit);
Emm42Motor ArmMotorZ(&arm3CommuInit, &arm3MotorInit);
bool ArmStopFlag = true;
float Position, Angle;
static float arm1Angle, arm2Angle,armZSpeed,armZPos;
void ArmStop() {
    ArmStopFlag = true;
    // TrayMotor.Stop();
     ArmMotor1.Stop();
      ArmMotor2.Stop();
     ArmMotorZ.Stop();
}

void ArmAngleCalc() {
  //   TrayMotor.SetTargetAngle(Angle);
        ArmMotor1.SetTargetAngle(-arm1Angle);
       ArmMotor2.SetTargetAngle(-arm2Angle);
    /*   if(armZPos > 0.7){
           ArmMotorZ.SetTargetPosition(2);
       }else if (armZPos < -0.7) {
           ArmMotorZ.SetTargetPosition(0);
       }*/
}
void AutoTraySet(uint8_t trayflag) {
    //TrayMotor.SetTargetPos(trayflag);
    CompleteTask();
}

void ArmSet(float Arm1Angle, float Arm2Angle,float ArmZPos) {
    ArmStopFlag = false;

    arm1Angle = Arm1Angle;
    arm2Angle = Arm2Angle;
    armZPos = ArmZPos;


}

void AutoArmSet(float armzPos,float arm1Pos,float arm2Pos){
    arm2Pos = arm2Pos * 180 / PI;
    if (arm1Pos > 1.2) {
        arm1Pos = 1.2f;
        bsp_BuzzerOn(500);
    } else if (arm1Pos < -1.2f) {
        arm1Pos = -1.2f;
        bsp_BuzzerOn(500);
    } else{
        bsp_BuzzerOff();
    }
    if (arm2Pos > 150) {
        arm2Pos = 150;
        bsp_BuzzerOn(1000);
    } else if (arm2Pos < -150) {
        arm2Pos = -150;
        bsp_BuzzerOn(1000);
    } else{
        bsp_BuzzerOff();
    }
    arm1Angle = arm1Pos;
    arm2Angle = arm2Pos;
    armZPos = armzPos;
  //  ArmMotor1.SetTargetAngle(arm1Pos);
   // ArmMotor2.SetTargetAngle(arm2Pos);
   // ArmMotorZ.SetTargetPosition(0);
}

void ARMHandle() {
    if (!ArmStopFlag) {
        ArmAngleCalc();
    }
}




