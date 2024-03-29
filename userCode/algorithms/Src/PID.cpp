//
// Created by LEGION on 2021/10/19.
//

#include "PID.h"

void PID::Reset() {
    PIDInfo.ref = 0;
    PIDInfo.fdb = 0;
    PIDInfo.err[0] = 0;
    PIDInfo.err[1] = 0;
    PIDInfo.errSum = 0;

    PIDInfo.componentKp = 0;
    PIDInfo.componentKi = 0;
    PIDInfo.componentKd = 0;

    PIDInfo.output = 0;

}

void PID::Reset(PID_Regulator_t *pidRegulator) {
    if (pidRegulator != nullptr)PIDInfo = *pidRegulator;
}

/**
 * 计算pid算法的控制量
 * @param target 目标量
 * @param feedback 反馈量
 * @return 控制量
 */
float PID::PIDCalc(float target, float feedback) {
    PIDInfo.fdb = feedback;
    PIDInfo.ref = target;
    PIDInfo.err[1] = PIDInfo.ref - PIDInfo.fdb;
    PIDInfo.componentKp = PIDInfo.err[1] * PIDInfo.kp;
    PIDInfo.errSum += PIDInfo.err[1];
    INRANGE(PIDInfo.errSum, -1 * PIDInfo.componentKiMax / PIDInfo.ki, PIDInfo.componentKiMax / PIDInfo.ki);
    PIDInfo.componentKi = PIDInfo.errSum * PIDInfo.ki;
    PIDInfo.componentKd = (PIDInfo.err[1] - PIDInfo.err[0]) * PIDInfo.kd;
    INRANGE(PIDInfo.componentKp, -1 * PIDInfo.componentKpMax, PIDInfo.componentKpMax);
    INRANGE(PIDInfo.componentKi, -1 * PIDInfo.componentKiMax, PIDInfo.componentKiMax);
    INRANGE(PIDInfo.componentKd, -1 * PIDInfo.componentKdMax, PIDInfo.componentKdMax);
    PIDInfo.output = PIDInfo.componentKp + PIDInfo.componentKi + PIDInfo.componentKd;
    INRANGE(PIDInfo.output, -1 * PIDInfo.outputMax, PIDInfo.outputMax);
    PIDInfo.err[0] = PIDInfo.err[1];
    return PIDInfo.output;
}

float PID::PIDCalc(float target, float feedback, float max) {
    PIDInfo.fdb = feedback;
    PIDInfo.ref = target;
    PIDInfo.outputMax = max;
    PIDInfo.err[1] = PIDInfo.ref - PIDInfo.fdb;
    PIDInfo.componentKp = PIDInfo.err[1] * PIDInfo.kp;
    PIDInfo.errSum += PIDInfo.err[1];
    INRANGE(PIDInfo.errSum, -1 * PIDInfo.componentKiMax / PIDInfo.ki, PIDInfo.componentKiMax / PIDInfo.ki);
    PIDInfo.componentKi = PIDInfo.errSum * PIDInfo.ki;
    PIDInfo.componentKd = (PIDInfo.err[1] - PIDInfo.err[0]) * PIDInfo.kd;
    INRANGE(PIDInfo.componentKp, -1 * PIDInfo.componentKpMax, PIDInfo.componentKpMax);
    INRANGE(PIDInfo.componentKi, -1 * PIDInfo.componentKiMax, PIDInfo.componentKiMax);
    INRANGE(PIDInfo.componentKd, -1 * PIDInfo.componentKdMax, PIDInfo.componentKdMax);
    PIDInfo.output = PIDInfo.componentKp + PIDInfo.componentKi + PIDInfo.componentKd;
    INRANGE(PIDInfo.output, -1 * PIDInfo.outputMax, PIDInfo.outputMax);
    PIDInfo.err[0] = PIDInfo.err[1];
    return PIDInfo.output;
}

float EASY_PID::PIDCalc(float target, float nowdata, float max) {
    float error = target - nowdata;
    integral = integral + error;
    out = kp * error + ki * integral + kd * (error - lasterror);
    lasterror = error;
    INRANGE(out, -1 * max, max);
    return out;
}
