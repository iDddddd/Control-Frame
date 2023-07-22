//
// Created by LEGION on 2021/10/4.
//
#include "ChassisTask.h"
#include "IMU.h"

constexpr float L = 0.24f; //车身长
constexpr float M = 0.24f; //车身宽

PID_Regulator_t pidRegulator1 = {//此为储存pid参数的结构体
        .kp = 0.3f,
        .ki = 0.004f,
        .kd = 0.0001f,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000
};
PID_Regulator_t pidRegulator2 = {//此为储存pid参数的结构体
        .kp = 0.01f,
        .ki = 0.0f,
        .kd = 0.0f,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000 //4010电机输出电流上限，可以调小，勿调大
};
PID_Regulator_t pidRegulator3 = {//此为储存pid参数的结构体
        .kp = 0.3f,
        .ki = 0.004f,
        .kd = 0.0001f,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000 //4010电机输出电流上限，可以调小，勿调大
};
MOTOR_INIT_t chassisMotorInit1 = {//底盘电机初始化结构体
        .speedPIDp = &pidRegulator1,
        .anglePIDp = nullptr,
        .ctrlType = SPEED_Single,
        .reductionRatio = 1.0f
};
MOTOR_INIT_t chassisMotorInit2 = {//底盘电机初始化结构体
        .speedPIDp = &pidRegulator2,
        .anglePIDp = nullptr,
        .ctrlType = SPEED_Single,
        .reductionRatio = 1.0f
};
MOTOR_INIT_t chassisMotorInit3 = {//底盘电机初始化结构体
        .speedPIDp = &pidRegulator3,
        .anglePIDp = nullptr,
        .ctrlType = SPEED_Single,
        .reductionRatio = 1.0f
};
MOTOR_INIT_t swerveMotorInit = {//底盘电机初始化结构体
        .speedPIDp = nullptr,
        .anglePIDp = nullptr,
        .ctrlType = DIRECT,
        .reductionRatio = 1.0f
};
COMMU_INIT_t chassisCommuInit1 = {
        ._id = 0x141,
        .canType = can1
};
COMMU_INIT_t chassisCommuInit2 = {
        ._id = 0x142,
        .canType = can1
};
COMMU_INIT_t chassisCommuInit3 = {
        ._id = 0x143,
        .canType = can1
};
COMMU_INIT_t chassisCommuInit4 = {
        ._id = 0x144,
        .canType = can1
};

//底盘电机实例化，之后只需调用SetTargetVelocity函数即可控制电机
//FOUR_Motor_4010 Classis_Motor(&chassisCommuInit1, &chassisCommuInit2, &chassisCommuInit3, &chassisCommuInit4,
//                              &chassisMotorInit1, &chassisMotorInit1, &chassisMotorInit3, &chassisMotorInit2);

Motor_4010 CFR(&chassisCommuInit1, &chassisMotorInit1);
Motor_4010 CFL(&chassisCommuInit2, &chassisMotorInit2);
Motor_4010 CBL(&chassisCommuInit3, &chassisMotorInit2);
Motor_4010 CBR(&chassisCommuInit4, &chassisMotorInit2);


Motor_4315 RFR(MOTOR_ID_1, &swerveMotorInit);
Motor_4315 RFL(MOTOR_ID_2, &swerveMotorInit);
Motor_4315 RBL(MOTOR_ID_3, &swerveMotorInit);
Motor_4315 RBR(MOTOR_ID_4, &swerveMotorInit);


AutoMove autoMove;
bool ChassisStopFlag = true;
float FBVelocity, LRVelocity, RTVelocity;
float ZeroYaw;


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

/**
 * @brief 无头模式速度设定
 *
 */
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

/**
 * @brief 自动移动设定速度
 */
void AutoSetVelocity() {
    ChassisStopFlag = false;
    autoMove.Handle();
    FBVelocity = autoMove.vx;
    LRVelocity = autoMove.vy;
    RTVelocity = 0;

}

void AutoChassisSet(float x, float y, float o) {
    autoMove.StartMove(x, y, o);
}

/**
 * @brief 自动模式下执行急停模式的底盘任务处理
 */
void AutoChassisStop() {
    ChassisStopFlag = true;

    //Classis_Motor.Stop();
    autoMove.StopMove();
    /*RFL.Stop();
    RFR.Stop();
    RBL.Stop();
    RBR.Stop();*/
    RFL.SetTargetAngle(0);
    RFR.SetTargetAngle(90);
    RBL.SetTargetAngle(90);
    RBR.SetTargetAngle(0);
    CFR.Stop();
    CFL.Stop();
    CBL.Stop();
    CBR.Stop();
}

/**
 * @brief 执行急停模式的底盘任务处理
 */
void ChassisStop() {
    ChassisStopFlag = true;

    //Classis_Motor.Stop();
    RFL.Stop();
    RFR.Stop();
    RBL.Stop();
    RBR.Stop();
    CFR.Stop();
    CFL.Stop();
    CBL.Stop();
    CBR.Stop();
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
   // rtVelocity *= -2.0f * PI;
    float vx, vy, w;
    vx = lrVelocity;
    vy = fbVelocity;
    w = rtVelocity * -2.0f * PI;
    float A, B, C, D;
    A = vx - w * L / 2;
    B = vx + w * L / 2;
    C = vy - w * M / 2;
    D = vy + w * M / 2;

    //计算四个轮子角度，单位：度
    RFLAngle = -atan2(B, D) * 180 / PI;
    RFRAngle = -atan2(B, C) * 180 / PI;
    RBRAngle = -atan2(A, C) * 180 / PI;
    RBLAngle = -atan2(A, D) * 180 / PI;

    //控制底盘电机角度
    RFL.SetTargetAngle(RFLAngle);
    RFR.SetTargetAngle(RFRAngle);
    RBL.SetTargetAngle(RBLAngle);
    RBR.SetTargetAngle(RBRAngle);

    //计算四个轮子线速度，单位：度/s
    ClassisSpeed[1] = sqrt(B * B + D * D)/(WHEEL_DIAMETER * PI) * 360 ;//左前轮
    ClassisSpeed[0] = -sqrt(B * B + C * C)/(WHEEL_DIAMETER * PI)* 360;//右前轮
    ClassisSpeed[3] = -sqrt(A * A + C * C)/(WHEEL_DIAMETER * PI) * 360;//右后轮
    ClassisSpeed[2] = sqrt(A * A + D * D)/(WHEEL_DIAMETER * PI) * 360;//左后轮

    //控制底盘电机转速
  //  Classis_Motor.SetTargetSpeed(ClassisSpeed);
    CFR.SetTargetSpeed(ClassisSpeed[0]);
    CFL.SetTargetSpeed(ClassisSpeed[1]);
    CBL.SetTargetSpeed(ClassisSpeed[2]);
    CBR.SetTargetSpeed(ClassisSpeed[3]);
}

