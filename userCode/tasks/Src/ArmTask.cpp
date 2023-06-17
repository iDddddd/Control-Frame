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
static float arm1Angle, arm2Angle,armZSpeed;
void ArmStop() {
    ArmStopFlag = true;
    // TrayMotor.Stop();
     ArmMotor1.Stop();
      ArmMotor2.Stop();
     ArmMotorZ.Stop();
}

void ArmAngleCalc() {
  //   TrayMotor.SetTargetAngle(Angle);
        ArmMotor1.SetTargetAngle(arm1Angle);
       ArmMotor2.SetTargetAngle(arm2Angle);
      ArmMotorZ.SetTargetSpeed(armZSpeed);
}
void AutoTraySet(uint8_t trayflag){
    //TrayMotor.SetTargetPos(trayflag);
    CompleteTask();
}
/*void ArmAngleCalc() {
    float arm1_angle = arm1Angle / 16384.0f * 2 * 3.1415f;
    float arm2_angle = arm2Angle / 16384.0f * 360.0f;
    arm1_angle -= PI;
    if (arm2_angle > 180) {
        arm2_angle -= 360;
    }
    if (arm1_angle > 1.2) {
        arm1_angle = 1.2f;
    } else if (arm1_angle < -1.2f) {
        arm1_angle = -1.2f;
    }
    if (arm2_angle > 90) {
        arm2_angle = 90;
    } else if (arm2_angle < -90) {
        arm2_angle = -90;
    }
    ArmMotor1.SetTargetAngle(arm1_angle);
   // ArmMotor2.SetTargetAngle(arm2_angle);

}*/

void ArmSet(float Arm1Angle, float Arm2Angle,float ArmZSpeed) {
    ArmStopFlag = false;

    arm1Angle = Arm1Angle;
    arm2Angle = Arm2Angle;
    armZSpeed = ArmZSpeed;

}

void AutoArmSet(uint16_t angle1, uint16_t angle2, uint8_t pos) {
    float arm1_angle = angle1 / 16384.0f * 2 * 3.1415926f;
    float arm2_angle = angle2 / 16384.0f * 360.0f;
    arm1_angle -= PI;
    /*if (arm2_angle > 180) {
        arm2_angle -= 360;
    }*/
    arm2_angle -= 180;
    if (arm1_angle > 1.2) {
        arm1_angle = 1.2f;
        bsp_BuzzerOn(500);
    } else if (arm1_angle < -1.2f) {
        arm1_angle = -1.2f;
        bsp_BuzzerOn(500);
    } else{
        bsp_BuzzerOff();
    }
    if (arm2_angle > 150) {
        arm2_angle = 150;
        bsp_BuzzerOn(1000);
    } else if (arm2_angle < -150) {
        arm2_angle = -150;
        bsp_BuzzerOn(1000);
    } else{
        bsp_BuzzerOff();
    }
    arm1Angle = arm1_angle;
    arm2Angle = arm2_angle;
    ArmMotor1.SetTargetAngle(arm1_angle);
    ArmMotor2.SetTargetAngle(arm2_angle);
    ArmMotorZ.SetTargetPosition(pos);
}

void ARMHandle() {
    if (!ArmStopFlag) {
        ArmAngleCalc();
    }
}




