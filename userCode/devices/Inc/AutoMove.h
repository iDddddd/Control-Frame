//
// Created by David9686 on 2022/12/15.
//

#ifndef RM_FRAME_C_AUTOMOVE_H
#define RM_FRAME_C_AUTOMOVE_H

#include "IMU.h"
#include "Device.h"
#include "ManiControl.h"
#include "KF.h"

/*结构体定义--------------------------------------------------------------*/
typedef struct {
    float d1;
    float d2;
    float d_max;
    float a;
    float v;
    float v_max;
} Pos_Para_t;

/*类型定义----------------------------------------------------------------*/
class Move_X {
public:
    float expectPos{0};
    Pos_Para_t Para{};
    float x_rel{};
    float v_rel{0};
    EASY_PID pid;
    bool FinishFlag{false};
    bool ReachFlag{0};

    uint16_t finishcount{0};  // ReachFlag达到后开始计数，计数结束后正式finish

    Move_X();

    ~Move_X() = default;

    bool stopFlag{false};

    void Calc(float target);

    float Handle(float reference);

    void Stop();

};

class Move_Y {
public:
    float lastreference{0};

    float expectPos{0};
    Pos_Para_t Para{};
    float y_rel{};
    float v_rel{0};
    EASY_PID pid;
    bool FinishFlag{false};
    bool ReachFlag{false};  // 到位后置1，用来增大kp值，使之稳定在目标点。

    uint16_t finishcount{0};  // ReachFlag达到后开始计数，计数结束后正式finish

    Move_Y();

    ~Move_Y() = default;

    bool stopFlag{false};

    void Calc(float target);

    float Handle(float reference);

    void Stop();

};

class Spin {
public:
    float expectPos{0};
    Pos_Para_t Para{};
    float o_rel{};
    float v_rel{0};
    EASY_PID pid;

    bool FinishFlag{false};//好像MoveX和MoveY里面都有，所以干脆这里也加上得了
    bool ReachFlag{0};

    uint16_t finishcount{0}; // ReachFlag达到后开始计数，计数结束后正式finish

    Spin();

    ~Spin() = default;

    bool stopFlag{false};

    void Calc(float target);

    float Handle(const float reference);

    void Stop();

};

class AutoMove {
public:

    float vx{};
    float vy{};
    float vo{};
    bool StopFlag{true};
    bool SendFlag{false};

    AutoMove() = default;

    void StartMove(float x_distance, float y_distance, float o_angle);

    void StopMove();

    ~AutoMove() = default;

    void Handle();


};


#endif //RM_FRAME_C_AUTOMOVE_H
