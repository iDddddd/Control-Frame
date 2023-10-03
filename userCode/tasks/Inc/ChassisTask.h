//
// Created by LEGION on 2021/10/4.
//

#ifndef RM_FRAME_C_CHASSISTASK_H
#define RM_FRAME_C_CHASSISTASK_H

#include "main.h"
#include "Motor.h"
#include "AutoMove.h"
#include "ChassisMotor.h"

#define WHEEL_DIAMETER             0.052f//4010直径 m
#define REDUCTION_RATIO 19.0f
#define TRACK_WIDTH            0.40f //轮距
#define WHEEL_BASE             0.38f //轴距
/*枚举类型定义------------------------------------------------------------*/

/*
 * 底盘任务错误信息枚举
 */
typedef enum {
    NONE
} CHASSIS_ERROR_E;

/*结构体定义--------------------------------------------------------------*/

/*
 * 底盘任务状态结构体
 */
typedef struct {

} CHASSIS_STATE_T;

/*
 * 底盘任务控制结构体
 */
typedef struct {


} CHASSIS_CONTROL_T;

/*底盘类*/
class Chassis{
private:
    Motor_4010 CFR;
    Motor_4010 CFL;
    Motor_4010 CBL;
    Motor_4010 CBR;
    Motor_4315 RFR;
    Motor_4315 RFL;
    Motor_4315 RBL;
    Motor_4315 RBR;
    
    float v8[8] = { 0,0,0,0,0,0,0,0 };// 4轮8速度
    
    bool ChassisStopFlag = true;
    float FBVelocity, LRVelocity, RTVelocity;
    float ZeroYaw;
public:
    Chassis(COMMU_INIT_t c1, MOTOR_INIT_t m1, COMMU_INIT_t c2, MOTOR_INIT_t m2, COMMU_INIT_t c3, MOTOR_INIT_t m3, COMMU_INIT_t c4, MOTOR_INIT_t m4, int ID1, MOTOR_INIT_t r1, int ID2, MOTOR_INIT_t r2, int ID3, MOTOR_INIT_t r3, int ID4, MOTOR_INIT_t r4);
    
    float* get_v8();

    void ChassisHandle();
    void ChassisSetVelocity(float _fbV, float _lrV, float _rtV);
    void HeadlessSetVelocity(float _fbV, float _lrV, float _rtV);
    void Headmemory();
    void HeadkeepSetVelocity(float _fbV, float _lrV, float _rtV);
    void AutoSetVelocity();
    void ChassisDistanceSet(float x, float y, float o);
    void ChassisVelocitySet(float x_vel, float y_vel, float w_vel);
    void AutoChassisStop();
    void ChassisStop();
    void WheelsSpeedCalc(float fbVelocity, float lrVelocity, float rtVelocity);
};

// 底盘对象实例化
extern Chassis chassis;

/*结构体成员取值定义组------------------------------------------------------*/
/*外部变量声明-------------------------------------------------------------*/

// extern CHASSIS_STATE_T chassisState;
// extern CHASSIS_CONTROL_T chassisControl;

// extern float FBVelocity, LRVelocity, RTVelocity;
// extern float ZeroYaw;


/*外部函数声明-------------------------------------------------------------
int sign(float x);

float SetAngle(float Angle);

void WheelAngleCalc(float fbVelocity, float lrVelocity, float rtVelocity);

void WheelsSpeedCalc(float fbVelocity, float lrVelocity, float rtVelocity);

void ChassisStop();

void ChassisSetVelocity(float _fbV, float _lrV, float _rtV);

void HeadlessSetVelocity(float _fbV, float _lrV, float _rtV);

void Headmemory();

void HeadkeepSetVelocity(float _fbV, float _lrV, float _rtV);

void AutoSetVelocity();*/

#endif //RM_FRAME_C_CHASSISTASK_H
