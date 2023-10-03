constexpr float TRACK_WIDTH = 0.24f; //轮距
constexpr float WHEEL_BASE = 0.24f; //轴距
constexpr float WHEEL_DIAMETER = 0.052f; //4010直径 m

#include "ChassisTask.h"
#include "IMU.h"

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

Chassis_State_t Chassis::SetTargetVelocity(Chassis_State_t set) {
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
}

void Chassis::ForwardKinematics() {
    for(auto& module : modules) {
        float vx = target.vx - target.w * module.posx;
        float vy = target.vy + target.w * module.posy;
        module.wheel.SetTargetSpeed(sqrt(vx * vx + vy * vy) / (WHEEL_DIAMETER * PI) * 360);
        module.swerve.SetTargetAngle(atan2(vy, vx) * 180 / PI + module.orient - module.zeroOffset);
    }
}

void Chassis::BackwardKinematics(){
    float sum_posx, sum_posy, sum_posL2 = 0;
    for(int i = 0; i < MODULE_NUM; i++){
        sum_posx += modules[i].posx;
        sum_posy += modules[i].posy;
        sum_posL2 += modules[i].posx * modules[i].posx + modules[i].posy * modules[i].posy;
    }
    float a = sum_posy, b = -sum_posx, c = sum_posL2;
    unsigned int n = MODULE_NUM;
    float rev[9] = {(n*c-b*b)/n/(n*c-a*a-b*b), a*b/n/(n*c-a*a-b*b), -a/(n*c-a*a-b*b),
                        a*b/n/(n*c-a*a-b*b), (n*c-a*a)/n/(n*c-a*a-b*b), -b/(n*c-a*a-b*b),
                        -a/(n*c-a*a-b*b), -b/(n*c-a*a-b*b), n/(n*c-a*a-b*b)};//ATA的逆矩阵
    estimation.vx = 0;
    estimation.vy = 0;
    estimation.w = 0;
    for(int i = 0; i < MODULE_NUM; i++){
        estimation.vx += ((rev[0] + rev[2] * modules[i].posy) * modules[i].vx + (rev[1] - rev[2] * modules[i].posx) * modules[i].vy);
        estimation.vy += ((rev[3] + rev[5] * modules[i].posy) * modules[i].vx + (rev[4] - rev[5] * modules[i].posx) * modules[i].vy);
        estimation.w += ((rev[6] + rev[8] * modules[i].posy) * modules[i].vx + (rev[7] - rev[8] * modules[i].posx) * modules[i].vy);
    }
}