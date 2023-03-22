//
// Created by David9686 on 2022/12/15.
//

#ifndef RM_FRAME_C_AUTOMOVE_H
#define RM_FRAME_C_AUTOMOVE_H

#include "IMU.h"
#include "Device.h"
#include "CatchControl.h"

class Move {
public:
    static float expectPos[3];
    float x_rel;
    uint8_t Index;
    float d1{};
    float d2{};
    float d_max{};
    float a;
    float v{};
    float v_max;
    float v_rel{};
    EASY_PID pid;
    static uint8_t FinishFlag;

    Move(uint8_t _n);
    ~Move();
    bool stopFlag{false};
    void Calc(float target);
    float Handle(float  &reference);
    float Handle_X(float  reference);
    float Handle_Y(float  reference);
    float Handle_O(const float  reference);

    void Stop();

};

class AutoMove {
public:
   // static float t;

    uint8_t num;
    float vx;
    float vy;
    float vo;
    bool StopFlag{false};
    AutoMove(uint8_t _num);
    void StartMove(float x_distance, float y_distance, float o_angle);
    void StopMove();
    ~AutoMove() = default;

    void Handle();


};


#endif //RM_FRAME_C_AUTOMOVE_H
