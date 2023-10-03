#include <cstdint>

constexpr std::size_t MODULE_NUM = 4;

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

COMMU_INIT_t chassisCommuInit1 = {0x141, can1};
COMMU_INIT_t chassisCommuInit2 = {0x142, can1};
COMMU_INIT_t chassisCommuInit3 = {0x143, can1};
COMMU_INIT_t chassisCommuInit4 = {0x144, can1};

constexpr float TRACK_WIDTH = 0.24f; //轮距
constexpr float WHEEL_BASE = 0.24f; //轴距
constexpr float WHEEL_DIAMETER = 0.052f; //4010直径 m

Chassis chassis;

Chassis::Chassis() : modules{ \
    Swerve_Module_t({{&chassisCommuInit1, &chassisMotorInit}, {MOTOR_ID_1, &swerveMotorInit}, 180, TRACK_WIDTH / 2, WHEEL_BASE / 2}), \
    Swerve_Module_t({{&chassisCommuInit2, &chassisMotorInit}, {MOTOR_ID_2, &swerveMotorInit}, 0, -TRACK_WIDTH / 2, WHEEL_BASE / 2}), \
    Swerve_Module_t({{&chassisCommuInit3, &chassisMotorInit}, {MOTOR_ID_3, &swerveMotorInit}, 0, -TRACK_WIDTH / 2, -WHEEL_BASE / 2}), \
    Swerve_Module_t({{&chassisCommuInit4, &chassisMotorInit}, {MOTOR_ID_4, &swerveMotorInit}, 180, TRACK_WIDTH / 2, -WHEEL_BASE / 2}),
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
        module.swerve.SetTargetAngle(atan2(vy, vx) * 180 / PI + module.theta);
    }
}
