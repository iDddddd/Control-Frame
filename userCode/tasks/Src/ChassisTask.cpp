constexpr float TRACK_WIDTH = 0.24f; //轮距
constexpr float WHEEL_BASE = 0.24f; //轴距
constexpr float WHEEL_DIAMETER = 0.052f; //4010直径 m
#include "ChassisTask.h"

Odometer_State_t Odometer::OdomReset() {
    Odometer_State_t retval = odom;
    odom = {0};
    return retval;
}

Odometer_State_t &Odometer::OdomCalc(Chassis_State_t curVel) {
#define INTERVAL 0.001f
    odom.x += (curVel.vx * cos(odom.theta) - curVel.vy * sin(odom.theta)) * INTERVAL;
    odom.y += (curVel.vx * sin(odom.theta) + curVel.vy * cos(odom.theta)) * INTERVAL;
    odom.theta += curVel.w * INTERVAL;
    return odom;
}

PID_Regulator_t pidRegulator1 = {//此为储存pid参数的结构体
        .kp = 0.38f,
        .ki = 0.006f,
        .kd = 0.3f,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000
};

MOTOR_INIT_t chassisMotorInit = {//底盘电机初始化结构体
        .speedPIDp = &pidRegulator1,
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

Chassis chassis;

Chassis::Chassis() : modules{ \
    Swerve_Module_t({{CAN1_ID(0x141), &chassisMotorInit}, {RS485_ID(0x00), &swerveMotorInit}, 0, 180, TRACK_WIDTH / 2, WHEEL_BASE / 2}), \
    Swerve_Module_t({{CAN1_ID(0x142), &chassisMotorInit}, {RS485_ID(0x01), &swerveMotorInit}, 0, 0, -TRACK_WIDTH / 2, WHEEL_BASE / 2}), \
    Swerve_Module_t({{CAN1_ID(0x143), &chassisMotorInit}, {RS485_ID(0x02), &swerveMotorInit}, 180, 180, -TRACK_WIDTH / 2, -WHEEL_BASE / 2}), \
    Swerve_Module_t({{CAN1_ID(0x144), &chassisMotorInit}, {RS485_ID(0x03), &swerveMotorInit}, 180,  0, TRACK_WIDTH / 2, -WHEEL_BASE / 2}),
    } {

}

Chassis& Chassis::Instance() {
    return chassis;
}

Chassis_State_t& Chassis::SetTargetVelocity(Chassis_State_t set) {
    target = set;
    return target;
}

void Chassis::Stop() {
    for(auto& module : modules) {
        module.swerve.Stop();
        module.wheel.Stop();
    }
    brake = true;
}

void Chassis::Handle() {
    if(!brake) {
        ForwardKinematics();
    }
    OdomCalc(BackwardEstimation());
}

void Chassis::ForwardKinematics() {
    for(auto& module : modules) {
        float vx = target.vx + target.w * module.posy;
        float vy = target.vy - target.w * module.posx;
        module.wheel.SetTargetSpeed(sqrt(vx * vx + vy * vy) / (WHEEL_DIAMETER * PI) * 360);
        module.swerve.SetTargetAngle(atan2(vx, vy) * 180 / PI + module.orient - module.zeroOffset);
    }
}

Chassis_State_t& Chassis::BackwardEstimation() {
    float sumX = 0, sumY = 0, sumL2 = 0;
    for(auto& module : modules){
        sumX += module.posx;
        sumY += module.posy;
        sumL2 += module.posx * module.posx + module.posy * module.posy;
    }

    #define DET (MODULE_NUM * (MODULE_NUM * sumL2 - sumX * sumX - sumY * sumY))

    float rev[3][3] = { \
        (MODULE_NUM * sumL2 - sumX * sumX) / DET, \
        -sumX * sumY / DET, \
        -sumY * MODULE_NUM  / DET, \
        -sumX * sumY / DET, \
        (MODULE_NUM * sumL2 - sumY * sumY) / DET, \
        MODULE_NUM * sumX / DET, \
        -sumY * MODULE_NUM / DET, \
        sumX * MODULE_NUM / DET, \
        MODULE_NUM * MODULE_NUM / DET \
    };//ATA的逆矩阵
    float sumVx = 0, sumVy = 0, crossProduct = 0;

    for(auto& module : modules){
        #define REAL_SPEED ((module.wheel.state.speed) / 360 * PI * WHEEL_DIAMETER)
        #define REAL_ANGLE (((module.swerve.nowAngle) + module.orient - module.zeroOffset) * PI / 180)  // Additional PI/2 should be considered in the following procedure
        float vx = REAL_SPEED * sin(REAL_ANGLE); // cos(-x + PI/2) = sin(x)，转向电机的nowAngle是顺时针旋转为正方向
        float vy = REAL_SPEED * cos(REAL_ANGLE); // sin(-x + PI/2) = cos(x)

        sumVx += vx;
        sumVy += vy;
        crossProduct += vx * module.posy - vy * module.posx;
    }

    estimation.vx = rev[0][0] * sumVx + rev[0][1] * sumVy + rev[0][2] * crossProduct;
    estimation.vy = rev[1][0] * sumVx + rev[1][1] * sumVy + rev[1][2] * crossProduct;
    estimation.w = rev[2][0] * sumVx + rev[2][1] * sumVy + rev[2][2] * crossProduct;

    return estimation;
}
