//
// Created by LEGION on 2021/10/4.
//
#include "ChassisTask.h"
#include "IMU.h"
#include "KF.h"
constexpr float L = 0.24f; //车身长
constexpr float M = 0.24f; //车身宽



PID_Regulator_t pidRegulator1 = {//此为储存pid参数的结构体
        .kp = 0.36f,
        .ki = 0.006f,
        .kd = 0.3f,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000
};
PID_Regulator_t pidRegulator2 = {//此为储存pid参数的结构体
        .kp = 0.45f,
        .ki = 0.006f,
        .kd = 0.5f,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000 //4010电机输出电流上限，可以调小，勿调大
};
PID_Regulator_t pidRegulator3 = {//此为储存pid参数的结构体
        .kp = 0.35f,
        .ki = 0.002f,
        .kd = 0.3f,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000 //4010电机输出电流上限，可以调小，勿调大
};
PID_Regulator_t pidRegulator4 = {//此为储存pid参数的结构体
        .kp = 0.41f,
        .ki = 0.006f,
        .kd = 0.4f,
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
MOTOR_INIT_t chassisMotorInit4 = {//底盘电机初始化结构体
        .speedPIDp = &pidRegulator4,
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

MOTOR_INIT_t swerveMotorInit2 = {//底盘电机初始化结构体
        .speedPIDp = nullptr,
        .anglePIDp = nullptr,
        .ctrlType = DIRECT,
        .reductionRatio = 1.0f
};
MOTOR_INIT_t swerveMotorInit3 = {//底盘电机初始化结构体
        .speedPIDp = nullptr,
        .anglePIDp = nullptr,
        .ctrlType = DIRECT,
        .reductionRatio = 1.0f
};
MOTOR_INIT_t swerveMotorInit4 = {//底盘电机初始化结构体
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


// 底盘对象实例化
Chassis chassis(chassisCommuInit1, chassisMotorInit1, chassisCommuInit2, chassisMotorInit2, chassisCommuInit3, chassisMotorInit3, chassisCommuInit4, chassisMotorInit4, MOTOR_ID_1, swerveMotorInit, MOTOR_ID_2, swerveMotorInit, MOTOR_ID_3, swerveMotorInit, MOTOR_ID_4, swerveMotorInit);

AutoMove autoMove;

// StateEstimator stateEstimator;


/*函数成员实现--------------------------------------------------*/
//构造函数
Chassis::Chassis(COMMU_INIT_t c1, MOTOR_INIT_t m1, COMMU_INIT_t c2, MOTOR_INIT_t m2, COMMU_INIT_t c3, MOTOR_INIT_t m3, COMMU_INIT_t c4, MOTOR_INIT_t m4, int ID1, MOTOR_INIT_t r1, int ID2, MOTOR_INIT_t r2, int ID3, MOTOR_INIT_t r3, int ID4, MOTOR_INIT_t r4):CFR(&c1, &m1), CFL(&c2, &m2), CBL(&c3, &m3), CBR(&c4, &m4), RFR(ID1, &r1), RFL(ID2, &r2), RBL(ID3, &r3), RBR(ID4, &r4){};

float* Chassis::get_v8(){
    return v8;
}

/**
 * @brief 底盘任务的处理函数，定时执行
 * @callergraph void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) in Device.cpp
 */
void Chassis::ChassisHandle() {
    // stateEstimator.UpdateState(v8);
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
void Chassis::ChassisSetVelocity(float _fbV, float _lrV, float _rtV) {
    ChassisStopFlag = false;
    FBVelocity = _fbV;
    LRVelocity = _lrV;
    RTVelocity = _rtV;
}

/**
 * @brief 无头模式速度设定
 *
 */
void Chassis::HeadlessSetVelocity(float _fbV, float _lrV, float _rtV) {
    ChassisStopFlag = false;
    FBVelocity = _fbV * cos(IMU::imu.attitude.yaw) - _lrV * sin(IMU::imu.attitude.yaw);
    LRVelocity = _fbV * sin(IMU::imu.attitude.yaw) + _lrV * cos(IMU::imu.attitude.yaw);
    RTVelocity = _rtV;
}

void Chassis::Headmemory() {
    ZeroYaw = IMU::imu.attitude.yaw;
}

void Chassis::HeadkeepSetVelocity(float _fbV, float _lrV, float _rtV) {
    ChassisStopFlag = false;
    FBVelocity = _fbV * cos((IMU::imu.attitude.yaw - ZeroYaw)) - _lrV * sin((IMU::imu.attitude.yaw - ZeroYaw));
    LRVelocity = _fbV * sin((IMU::imu.attitude.yaw - ZeroYaw)) + _lrV * cos((IMU::imu.attitude.yaw - ZeroYaw));
    RTVelocity = _rtV;
}

/**
 * @brief 自动移动设定速度
 */
void Chassis::AutoSetVelocity() {
    ChassisStopFlag = false;
    autoMove.Handle();
    FBVelocity = autoMove.vy;
    LRVelocity = autoMove.vx;
    RTVelocity = autoMove.vo;

}

void Chassis::ChassisDistanceSet(float x, float y, float o) {
    autoMove.StartMove(x, y, o);
}


void Chassis::ChassisVelocitySet(float x_vel, float y_vel, float w_vel) {
    FBVelocity = y_vel;
    LRVelocity = x_vel;
    RTVelocity = w_vel;
}
/**
 * @brief 自动模式下执行急停模式的底盘任务处理
 */
void Chassis::AutoChassisStop() {
    ChassisStopFlag = true;

    //Classis_Motor.Stop();
    autoMove.StopMove();
    /*chassis.RFL.Stop();
    chassis.RFR.Stop();
    chassis.RBL.Stop();
    chassis.RBR.Stop();*/
    RFL.SetTargetAngle(0);
    RFR.SetTargetAngle(90);
    RBL.SetTargetAngle(90);//让车辆能及时刹住
    RBR.SetTargetAngle(0);
    CFR.Stop();
    CFL.Stop();
    CBL.Stop();
    CBR.Stop();
}

/**
 * @brief 执行急停模式的底盘任务处理
 */
void Chassis::ChassisStop() {
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
void Chassis::WheelsSpeedCalc(float fbVelocity, float lrVelocity, float rtVelocity) {
    float ClassisSpeed[4];
    float RFLAngle, RFRAngle, RBLAngle, RBRAngle;
   // rtVelocity *= -2.0f * PI;
    float vx, vy, w;
    vx = lrVelocity;
    vy = fbVelocity;
    w = rtVelocity * 2.0f * PI;//w顺时针为正
    float A, B, C, D;
    A = vx - w * L / 2;
    B = vx + w * L / 2;
    C = vy - w * M / 2;
    D = vy + w * M / 2;

    //计算四个轮子角度，单位：度
    RFRAngle = atan2(B, C) * 180 / PI;
    RFLAngle = atan2(B, D) * 180 / PI; 
    RBLAngle = atan2(A, D) * 180 / PI;
    RBRAngle = atan2(A, C) * 180 / PI;


    //计算四个轮子线速度，单位：度/s
    
    ClassisSpeed[0] = -sqrt(B * B + C * C)/(WHEEL_DIAMETER * PI)* 360;//右前轮
    ClassisSpeed[1] = sqrt(B * B + D * D)/(WHEEL_DIAMETER * PI) * 360 ;//左前轮
    ClassisSpeed[2] = sqrt(A * A + D * D)/(WHEEL_DIAMETER * PI) * 360;//左后轮
    ClassisSpeed[3] = -sqrt(A * A + C * C)/(WHEEL_DIAMETER * PI) * 360;//右后轮

/*修正角度*/   
/*
    if(abs(int(chassis.RFRAngle - chassis.RFR.nowAngle) % 360) >= 90) {
        ClassisSpeed[0] = -ClassisSpeed[0];
        chassis.RFRAngle = (chassis.RFRAngle > 0) ? (chassis.RFRAngle - 180) : (chassis.RFRAngle + 180);
    }
    if(abs(int(chassis.RFLAngle - chassis.RFL.nowAngle) % 360) >= 90) {
        ClassisSpeed[1] = -ClassisSpeed[1];
        chassis.RFLAngle = (chassis.RFLAngle > 0) ? (chassis.RFLAngle - 180) : (chassis.RFLAngle + 180);
    }
    if(abs(int(chassis.RBLAngle - chassis.RBL.nowAngle) % 360) >= 90) {
        ClassisSpeed[2] = -ClassisSpeed[2];
        chassis.RBLAngle = (chassis.RBLAngle > 0) ? (chassis.RBLAngle - 180) : (chassis.RBLAngle + 180);
    }
    if(abs(int(chassis.RBRAngle - chassis.RBR.nowAngle) % 360) >= 90) {
        ClassisSpeed[3] = -ClassisSpeed[3];
        chassis.RBRAngle = (chassis.RBRAngle > 0) ? (chassis.RBRAngle - 180) : (chassis.RBRAngle + 180);
    }*/  

    //设置底盘电机角度
    RFL.SetTargetAngle(RFLAngle);
    RFR.SetTargetAngle(RFRAngle);
    RBL.SetTargetAngle(RBLAngle);
    RBR.SetTargetAngle(RBRAngle);

    //设置底盘电机转速
    CFR.SetTargetSpeed(ClassisSpeed[0]);
    CFL.SetTargetSpeed(ClassisSpeed[1]);
    CBL.SetTargetSpeed(ClassisSpeed[2]);
    CBR.SetTargetSpeed(ClassisSpeed[3]);

    //注：编码器的速度以及遥控器输入的控制速度均为°/s，用m/s时可能需要改pid参数
    v8[0] = CFR.state.speed * sin(RFR.nowAngle/180*PI);// / 180 * PI * WHEEL_DIAMETER
    v8[1] = CFR.state.speed * cos(RFR.nowAngle/180*PI);
    v8[2] = CFL.state.speed * sin(RFL.nowAngle/180*PI);
    v8[3] = CFL.state.speed * cos(RFL.nowAngle/180*PI);
    v8[4] = CBL.state.speed * sin(RBL.nowAngle/180*PI);
    v8[5] = CBL.state.speed * cos(RBL.nowAngle/180*PI);
    v8[6] = CBR.state.speed * sin(RBR.nowAngle/180*PI);
    v8[7] = CBR.state.speed * cos(RBR.nowAngle/180*PI);
    // CFR.target_vx = CFR.targetSpeed * sin(RFRAngle/180*PI);
    // CFR.target_vy = CFR.targetSpeed * cos(RFRAngle/180*PI);
    // CFL.target_vx = CFL.targetSpeed * sin(RFLAngle/180*PI);
    // CFL.target_vy = CFL.targetSpeed * cos(RFLAngle/180*PI);
    // CBL.target_vx = CBL.targetSpeed * sin(RBLAngle/180*PI);
    // CBL.target_vy = CBL.targetSpeed * cos(RBLAngle/180*PI);
    // CBR.target_vx = CBR.targetSpeed * sin(RBRAngle/180*PI);
    // CBR.target_vy = CBR.targetSpeed * cos(RBRAngle/180*PI);

}

