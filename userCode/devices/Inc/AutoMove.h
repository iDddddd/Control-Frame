//
// Created by David9686 on 2022/12/15.
//

#ifndef RM_FRAME_C_AUTOMOVE_H
#define RM_FRAME_C_AUTOMOVE_H

#include "IMU.h"
#include "Device.h"

class Move {
public:
    static float expectPos[3];
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

    Move();
    ~Move();
    bool stopFlag{true};
    void Calc(float target);
    void Handle(float  &reference);
/*    void Handle_X();
    void Handle_Y();
    void Handle_O();*/

    void Stop();

};
class AutoMove {
public:
    static float t;
    Move x;
    Move y;
    Move o;
    bool StopFlag{true};
    AutoMove();
    void StartMove(float x_distance, float y_distance, float o_angle);
    void StopMove();
    ~AutoMove() = default;

    void Handle();


};


#endif //RM_FRAME_C_AUTOMOVE_H
