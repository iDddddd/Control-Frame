//
// Created by David9686 on 2022/12/15.
//

#ifndef RM_FRAME_C_AUTOMOVE_H
#define RM_FRAME_C_AUTOMOVE_H

#include "IMU.h"
#include "Device.h"

class Move {
public:
    float d1;
    float d2;
    float d_max;
    float a = 2;
    float v;
    float v_max = 1;
    float v_rel;

    Move();
    ~Move();

    void Calc(float target);

    void Handle();
};
class AutoMove {
public:
    static float t;
    Move x;
    Move y;
    Move o;

    AutoMove(float x_distance, float y_distance, float o_angle);

    ~AutoMove() = default;

    void Handle();


};


#endif //RM_FRAME_C_AUTOMOVE_H
