//
// Created by LEGION on 2021/10/4.
//
#include "ChassisTask.h"
#include "IMU.h"

constexpr float L = 0.24f; //车身长
constexpr float M = 0.24f; //车身宽

PID_Regulator_t pidRegulator1 = {//此为储存pid参数的结构体，四个底盘电机共用
        .kp = -0.19f,
        .ki = -0.0004f,
        .kd = 0,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000
};
PID_Regulator_t pidRegulator2 = {//此为储存pid参数的结构体，四个底盘电机共用
        .kp = 0.19f,
        .ki = 0.0004f,
        .kd = 0,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000 //4010电机输出电流上限，可以调小，勿调大
};
MOTOR_INIT_t chassisMotorInit1 = {//四个底盘电机共用的初始化结构体
        .speedPIDp = &pidRegulator1,
        .anglePIDp = nullptr,
        .reductionRatio = 1.0f
};
MOTOR_INIT_t chassisMotorInit2 = {//四个底盘电机共用的初始化结构体
        .speedPIDp = &pidRegulator2,
        .anglePIDp = nullptr,
        .reductionRatio = 1.0f
};

MOTOR_INIT_t swerveMotorInit = {//四个底盘电机共用的初始化结构体
        .speedPIDp = nullptr,
        .anglePIDp = nullptr,
        .reductionRatio = 1.0f
};
COMMU_INIT_t chassisCommuInit1 = {
        ._id = 0x141,
        .ctrlType = SPEED_Single,
        .canType = can1
};
COMMU_INIT_t chassisCommuInit2 = {
        ._id = 0x142,
        .ctrlType = SPEED_Single,
        .canType = can1
};
COMMU_INIT_t chassisCommuInit3 = {
        ._id = 0x143,
        .ctrlType = SPEED_Single,
        .canType = can1
};
COMMU_INIT_t chassisCommuInit4 = {
        ._id = 0x144,
        .ctrlType = SPEED_Single,
        .canType = can1
};


FOUR_Motor_4010 Classis_Motor(&chassisCommuInit1, &chassisCommuInit2, &chassisCommuInit3, &chassisCommuInit4,
                              &chassisMotorInit1, &chassisMotorInit2);

Motor_4315 RFL(MOTOR_ID_1, &swerveMotorInit);
Motor_4315 RFR(MOTOR_ID_2, &swerveMotorInit);
Motor_4315 RBR(MOTOR_ID_3, &swerveMotorInit);
Motor_4315 RBL(MOTOR_ID_4, &swerveMotorInit);

//Move move(2.0);
bool ChassisStopFlag = true;
float FBVelocity, LRVelocity, RTVelocity;
float ZeroYaw;

void ChassisStart() {

}

/**
 * @brief 底盘任务的处理函数，定时执行
 * @callergraph void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) in Device.cpp
 */
void ChassisHandle() {
    if (!ChassisStopFlag) {
        WheelsSpeedCalc(FBVelocity, LRVelocity, RTVelocity);
    }

}

/**
 * @brief 用于控制任务控制底盘速度
 * @param _fbV 底盘前后方向速度
 * @param _lrV 底盘左右方向速度
 * @param _rtV 底盘旋转速度
 */
void ChassisSetVelocity(float _fbV, float _lrV, float _rtV) {
    ChassisStopFlag = false;
    FBVelocity = _fbV;
    LRVelocity = _lrV;
    RTVelocity = _rtV;
}

void HeadlessSetVelocity(float _fbV, float _lrV, float _rtV) {
    ChassisStopFlag = false;
    FBVelocity = _fbV * cos(IMU::imu.attitude.yaw) - _lrV * sin(IMU::imu.attitude.yaw);
    LRVelocity = _fbV * sin(IMU::imu.attitude.yaw) + _lrV * cos(IMU::imu.attitude.yaw);
    RTVelocity = _rtV;
}

void Headmemory() {
    ZeroYaw = IMU::imu.attitude.yaw;
}

void HeadkeepSetVelocity(float _fbV, float _lrV, float _rtV) {
    ChassisStopFlag = false;
    FBVelocity = _fbV * cos((IMU::imu.attitude.yaw - ZeroYaw)) - _lrV * sin((IMU::imu.attitude.yaw - ZeroYaw));
    LRVelocity = _fbV * sin((IMU::imu.attitude.yaw - ZeroYaw)) + _lrV * cos((IMU::imu.attitude.yaw - ZeroYaw));
    RTVelocity = _rtV;
}

void AutoSetVelocity() {
    ChassisStopFlag = false;
  //  move.Handle();
    if (IMU::imu.position.displace[1] < 2) {
   //     FBVelocity = move.v_rel;
        LRVelocity = 0;
        RTVelocity = 0;
    } else {
        FBVelocity = 0;
        LRVelocity = 0;
        RTVelocity = 0;
    }

}

/**
 * @brief 执行急停模式的底盘任务处理
 */
void ChassisStop() {
    ChassisStopFlag = true;

    Classis_Motor.Stop();
    RFL.Stop();
    RFR.Stop();
    RBL.Stop();
    RBR.Stop();
}

int sign(float x) {
    if (x < 0) return -1;
    if (x > 0) return 1;
    return 0;
}

float SetAngle(float Angle) {
    if (Angle > 180.0) {
        Angle -= 180.0;
    }
    if (Angle <= 0) {
        Angle += 180.0;
    }
    return Angle;
}

/**
 * @brief 速度与角度计算任务
 * @param fbVelocity
 * @param lrVelocity
 * @param rtVelocity
 */
void WheelsSpeedCalc(float fbVelocity, float lrVelocity, float rtVelocity) {
    float ClassisSpeed[4];
    float RFLAngle, RFRAngle, RBLAngle, RBRAngle;
    rtVelocity = -RPM2RADpS(rtVelocity);

    //计算四个轮子线速度，单位：m/s
    /**
     * @brief 此处四句代码需要结合底盘的三个速度，计算处四个轮子的位置对应的线速度。
     * @param fbVelocity,lrVelocity,rtVelocity
     * @return CMFLSpeed CMFRSpeed CMBLSpeed CMBRSpeed
     */
//    CMFLSpeed = fbVelocity - rtVelocity;
//    CMFRSpeed = -fbVelocity - rtVelocity;
//    CMBLSpeed = -fbVelocity + rtVelocity;
//    CMBRSpeed = fbVelocity + rtVelocity;

    RFLAngle = -atan2((lrVelocity - rtVelocity * L / 2), (fbVelocity - rtVelocity * M / 2)) * 180 / 3.1415926f;
    RFRAngle = -atan2((lrVelocity - rtVelocity * L / 2), (fbVelocity + rtVelocity * M / 2)) * 180 / 3.1415926f;
    RBLAngle = -atan2((lrVelocity + rtVelocity * L / 2), (fbVelocity - rtVelocity * M / 2)) * 180 / 3.1415926f;
    RBRAngle = -atan2((lrVelocity + rtVelocity * L / 2), (fbVelocity + rtVelocity * M / 2)) * 180 / 3.1415926f;

    //控制底盘电机角度
    RFL.SetTargetAngle(RFLAngle);
    RFR.SetTargetAngle(RFRAngle);
    RBL.SetTargetAngle(RBLAngle);
    RBR.SetTargetAngle(RBRAngle);

    ClassisSpeed[0] = ((/*-sign(fbVelocity - rtVelocity * M / 2) */
                        sqrt((lrVelocity - rtVelocity * L / 2) * (lrVelocity - rtVelocity * L / 2) +
                             (fbVelocity - rtVelocity * M / 2) * (fbVelocity - rtVelocity * M / 2))) /
                       (WHEEL_DIAMETER / 2.0f)) * 180 / 3.1415926f;//左前轮
    ClassisSpeed[1] = -((/*sign(fbVelocity + rtVelocity * M / 2) */
                        sqrt((lrVelocity - rtVelocity * L / 2) * (lrVelocity - rtVelocity * L / 2) +
                             (fbVelocity + rtVelocity * M / 2) * (fbVelocity + rtVelocity * M / 2))) /
                       (WHEEL_DIAMETER / 2.0f)) * 180 / 3.1415926f;//右前轮
    ClassisSpeed[2] = -((/*sign(fbVelocity + rtVelocity * M / 2) */
                        sqrt((lrVelocity + rtVelocity * L / 2) * (lrVelocity + rtVelocity * L / 2) +
                             (fbVelocity + rtVelocity * M / 2) * (fbVelocity + rtVelocity * M / 2))) /
                       (WHEEL_DIAMETER / 2.0f)) * 180 / 3.1415926f;//右后轮
    ClassisSpeed[3] = ((/*-sign(fbVelocity - rtVelocity * M / 2) */
                        sqrt((lrVelocity + rtVelocity * L / 2) * (lrVelocity + rtVelocity * L / 2) +
                             (fbVelocity - rtVelocity * M / 2) * (fbVelocity - rtVelocity * M / 2))) /
                       (WHEEL_DIAMETER / 2.0f)) * 180 / 3.1415926f;//左后轮

    //控制底盘电机转速
    Classis_Motor.SetTargetSpeed(ClassisSpeed);
}

