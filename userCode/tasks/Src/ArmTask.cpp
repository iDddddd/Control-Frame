//
// Created by David9686 on 2022/12/5.
//

#include "ArmTask.h"




MOTOR_INIT_t arm1MotorInit = {//四个底盘电机共用的初始化结构体
        .speedPIDp = nullptr,
        .anglePIDp = nullptr,
        ._motorID = MOTOR_ID_1,
        .reductionRatio = 1.0f,
        .ctrlType = DIRECT,
        .commuType = ARMCAN1,
};
MOTOR_INIT_t arm2MotorInit = {//四个底盘电机共用的初始化结构体
        .speedPIDp = nullptr,
        .anglePIDp = nullptr,
        ._motorID = MOTOR_ID_1,
        .reductionRatio = 1.0f,
        .ctrlType = DIRECT,
        .commuType = ARMCAN2,
};


Motor ARM1(MOTOR_ID_1, &arm1MotorInit);
Motor ARM2(MOTOR_ID_1, &arm2MotorInit);
