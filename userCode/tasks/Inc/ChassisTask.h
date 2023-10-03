//
// Created by LEGION on 2021/10/4.
//

#ifndef RM_FRAME_C_CHASSISTASK_H
#define RM_FRAME_C_CHASSISTASK_H

#include "main.h"
#include "Motor.h"
#include "AutoMove.h"
#include "ChassisMotor.h"

/*枚举类型定义------------------------------------------------------------*/

/*
 * 底盘任务错误信息枚举
 */
typedef enum {
    NONE
} CHASSIS_ERROR_E;

/*结构体定义--------------------------------------------------------------*/

/**
 * 底盘任务状态结构体
 */
typedef struct {
    float vx;
    float vy;
    float w;
} Chassis_State_t;

typedef struct {
    Motor_4010 wheel;
    Motor_4315 swerve;
    float theta;
    float posx, posy;
} Swerve_Module_t;

class Chassis {
public:
    Chassis();
    static Chassis& Instance();

    Chassis_State_t SetTargetVelocity(Chassis_State_t set);
    void Stop();

    void Handle();

private:
    Swerve_Module_t modules[MODULE_NUM];
    Chassis_State_t target = {0};
    Chassis_State_t estimation = {0};
    bool brake = false;
/*    const Swerve_Module_t* FR = modules;
    const Swerve_Module_t* FL = modules + 1;
    const Swerve_Module_t* BL = modules + 2;
    const Swerve_Module_t* BR = modules + 3;*/

    void ForwardKinematics();
};


/*外部函数声明-------------------------------------------------------------*/
void WheelsSpeedCalc(float fbVelocity, float lrVelocity, float rtVelocity);

#endif //RM_FRAME_C_CHASSISTASK_H
