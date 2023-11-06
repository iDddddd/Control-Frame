#ifndef RM_FRAME_C_CHASSISTASK_H
#define RM_FRAME_C_CHASSISTASK_H

#include "main.h"
#include "Motor.h"
#include "ChassisMotor.h"

constexpr std::size_t MODULE_NUM = 4u;

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
    float w;// 逆时针为正
} Chassis_State_t;

typedef struct {
    float x;
    float y;
    float theta;
} Odometer_State_t;

typedef struct {
    Motor_4010 wheel;
    Motor_4315 swerve;
    float orient, zeroOffset; // orient指轮组安装朝向，车轮向前为零点，逆时针为正；zeroOffset指电机零点的位置，电路板朝右为零点，与电机定义相同顺时针为正
    float posx, posy;
} Swerve_Module_t;

class Odometer {
public:
    Odometer_State_t OdomReset();
    Odometer_State_t &OdomCalc(Chassis_State_t curVel);
    Odometer_State_t getOdom();
protected:
    Odometer_State_t odom{0};
};

class Chassis : public Odometer{
public:
    Chassis();
    static Chassis& Instance();

    Chassis_State_t& SetTargetVelocity(Chassis_State_t set);

    void Stop();

    void Handle();

private:
    Swerve_Module_t modules[MODULE_NUM];
    Chassis_State_t target = {0};
    Chassis_State_t estimation = {0};
    Chassis_State_t error = {0};
    bool brake = false;
/*    const Swerve_Module_t* FR = modules;
    const Swerve_Module_t* FL = modules + 1;
    const Swerve_Module_t* BL = modules + 2;
    const Swerve_Module_t* BR = modules + 3;*/

    void ForwardKinematics();
    Chassis_State_t& BackwardEstimation();
};

#endif //RM_FRAME_C_CHASSISTASK_H
