//
// Created by LEGION on 2021/10/19.
//

#ifndef RM_FRAME_C_PID_H
#define RM_FRAME_C_PID_H

#include "Device.h"

typedef struct PID_Regulator_t {
    float ref;
    float fdb;
    float err[2];
    float errSum;
    float kp;
    float ki;
    float kd;
    float componentKp;
    float componentKi;
    float componentKd;
    float componentKpMax;
    float componentKiMax;
    float componentKdMax;
    float output;
    float outputMax;
} PID_Regulator_t;

class PID {
public:

    PID_Regulator_t PIDInfo{};

    void Reset(PID_Regulator_t *pidRegulator);

    void Reset();

    float PIDCalc(float target, float feedback);

    float PIDCalc(float target, float feedback, float max);
};

class EASY_PID {
public:
    float target = 0;
    float out = 0;
    float integral = 0;
    float lasterror;
    float kp, kd, ki;
    float PIDCalc(float target, float nowdata,float max);

};
#endif //RM_FRAME_C_PID_H
