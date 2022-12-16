//
// Created by David9686 on 2022/12/15.
//

#ifndef RM_FRAME_C_AUTOMOVE_H
#define RM_FRAME_C_AUTOMOVE_H
#include "IMU.h"
#include "Device.h"

class Move: public Device
{
    static float t;
    static float v;
    static float x;
    float d1 = 0;
    float d2 = 0;
    float d_max;
    static float a;
    static float v_max;

public:

    float v_rel ;
    Move(float d);
    ~Move();
    void Handle() override;
    void ErrorHandle() override;

};


#endif //RM_FRAME_C_AUTOMOVE_H
