//
// Created by mac on 2022/12/14.
//

#include "ArmTask.h"


MOTOR_TYPE_e arm1Type = ARM1;
MOTOR_TYPE_e arm2Type = ARM2;

PID_Regulator_t pidRegulator3 = {//此为储存pid参数的结构体，四个底盘电机共用
        .kp = -1.0f,
        .ki = 0.0f,
        .kd = 0,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000 //3508电机输出电流上限，可以调小，勿调大
};
PID_Regulator_t pidRegulator4 = {//此为储存pid参数的结构体，四个底盘电机共用
        .kp = -1.0f,
        .ki = 0.0f,
        .kd = 0,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000 //3508电机输出电流上限，可以调小，勿调大
};
MOTOR_INIT_t chassisMotorInit3 = {//四个底盘电机共用的初始化结构体
        .speedPIDp = &pidRegulator3,
        .anglePIDp = &pidRegulator4,
        ._motorID = MOTOR_ID_1,
        .reductionRatio = 1.0f,
        .ctrlType = POSITION_Double,
        .commuType = CAN,
};

ARMMotor ARMMotor1(&arm1Type);
ARMMotor ARMMotor2(&arm2Type);
Motor ARMMotor3(MOTOR_ID_5,&chassisMotorInit3);

bool ArmStopFlag = true;

void ARMHandle(){
    ARMMotor3.SetTargetAngle(0);
    //ARMMotor1.Handle();
    ARMMotor2.Handle();
   // ARMMotor3.Handle();
}




