//
// Created by David9686 on 2022/12/15.
//

#include "AutoMove.h"

Move_X X;
Move_Y Y;
Spin O;


void AutoMove::Handle() {
    if (!StopFlag) {
        vx = X.Handle(IMU::imu.position.displace[1]);
        vy = Y.Handle(IMU::imu.position.displace[0]);
        vo = O.Handle(IMU::imu.attitude.yaw);
    } else {
        AutoChassisStop();
    }
    if (X.FinishFlag && Y.FinishFlag && !SendFlag) {
        StopMove();
        CompleteTask();
        SendFlag = true;
    }//完成后发送
}

void AutoMove::StartMove(float x_distance, float y_distance, float o_angle) {
    StopFlag = false;
    SendFlag = false;
    X.Calc(x_distance);
    Y.Calc(y_distance);
  //  O.Calc(o_angle);
    IMU::imu.position.displace[0] = 0;
    IMU::imu.position.displace[1] = 0;
}

void AutoMove::StopMove() {
    StopFlag = true;
    X.Stop();
    Y.Stop();
    O.Stop();
}


Move_X::Move_X() {
    Para.a = 1.5;
    Para.v_max = 2;
    pid.kp = 1;
    pid.ki = 0;
    pid.kd = 0;
}

void Move_X::Calc(float target) {
    stopFlag = false;
    FinishFlag = false;
    expectPos = 0;
    Para.d_max = target;
    Para.d1 = Para.v_max * Para.v_max / (2 * Para.a);
    Para.d2 = target - 2 * Para.d1;
    if (Para.d2 < 0) {
        Para.d1 = target / 2;
        Para.d2 = 0;
        Para.v_max = sqrt(2 * Para.a * Para.d1);
    }
}

float Move_X::Handle(float reference) {
    if (stopFlag) {
        v_rel = 0;
    } else {
        if (expectPos >= Para.d_max) {
            Para.v = 0;
        } else if (expectPos < Para.d1) {
            Para.v += Para.a * 0.001f;
            expectPos += (2 * Para.v - Para.a * 0.001f) / 2 * 0.001f;
        } else if (expectPos > (Para.d1 + Para.d2)) {
            Para.v -= Para.a * 0.001f;
            expectPos += (2 * Para.v + Para.a * 0.001f) / 2 * 0.001f;
        } else if (expectPos > Para.d1 && expectPos < (Para.d1 + Para.d2)) {
            expectPos += Para.v * 0.001f;
        }

        v_rel = Para.v + pid.PIDCalc(expectPos, reference, 2.0);
        if (v_rel > Para.v_max) {
            v_rel = Para.v_max;
        }
    }
    if (reference >= Para.d_max - 0.05) {
        //if (expectPos >= Para.d_max) {
            Stop();
            FinishFlag = true;
        }
    return v_rel;
}

void Move_X::Stop() {
    stopFlag = true;
}


Move_Y::Move_Y() {
    Para.a = 1.5;
    Para.v_max = 2;
    pid.kp = 1;
    pid.ki = 0;
    pid.kd = 0;

}

void Move_Y::Calc(float target) {
    stopFlag = false;
    FinishFlag = false;
    expectPos = 0;
    Para.d_max = target;
    Para.d1 = Para.v_max * Para.v_max / (2 * Para.a);
    Para.d2 = target - 2 * Para.d1;
    if (Para.d2 < 0) {
        Para.d1 = target / 2;
        Para.d2 = 0;
        Para.v_max = sqrt(2 * Para.a * Para.d1);
    }

}

float Move_Y::Handle(float reference) {
    if (stopFlag) {
        v_rel = 0;
    } else {
        if (expectPos >= Para.d_max) {
            Para.v = 0;
        } else if (expectPos < Para.d1) {
            Para.v += Para.a * 0.001f;
            expectPos += (2 * Para.v - Para.a * 0.001f) / 2 * 0.001f;
        } else if (expectPos > (Para.d1 + Para.d2)) {
            Para.v -= Para.a * 0.001f;
            expectPos += (2 * Para.v + Para.a * 0.001f) / 2 * 0.001f;
        } else if (expectPos > Para.d1 && expectPos < (Para.d1 + Para.d2)) {
            expectPos += Para.v * 0.001f;
        }

        v_rel = Para.v + pid.PIDCalc(expectPos, reference, 2.0);
        if (v_rel > Para.v_max) {
            v_rel = Para.v_max;
        }
        if (reference >= Para.d_max - 0.05) {
        //if (expectPos >= Para.d_max) {
            Stop();
            FinishFlag = true;
        }
    }
    return v_rel;
}

void Move_Y::Stop() {
    stopFlag = true;

}

Spin::Spin() {
    Para.a = 1;
    Para.v_max = 0.5;
    pid.kp = 0.015;
    pid.ki = 0;
    pid.kd = 0;

}

void Spin::Calc(float target) {
    stopFlag = false;
    Para.d_max = target-IMU::imu.attitude.yaw;
    Para.d1 = Para.v_max * Para.v_max / (2 * Para.a);
    Para.d2 = target - 2 * Para.d1;
    if (Para.d2 < 0) {
        Para.d1 = target / 2;
        Para.d2 = 0;
        Para.v_max = sqrt(2 * Para.a * Para.d1);
    }
}

float Spin::Handle(const float reference) {
    if (stopFlag) {
        v_rel = 0;
    } else {
        if (expectPos >= Para.d_max) {
            Para.v = 0;
        } else if (expectPos < Para.d1) {
            Para.v += Para.a * 0.001f;
            expectPos += (2 * Para.v - Para.a * 0.001f) / 2 * 0.001f;
        } else if (expectPos > (Para.d1 + Para.d2)) {
            Para.v -= Para.a * 0.001f;
            expectPos += (2 * Para.v + Para.a * 0.001f) / 2 * 0.001f;
        } else if (expectPos > Para.d1 && expectPos < (Para.d1 + Para.d2)) {
            expectPos += Para.v * 0.001f;
        }
    if(abs(expectPos-reference) < 0.01){
        return 0;
    }
        v_rel = Para.v + pid.PIDCalc(expectPos, reference, 2.0);
        if (v_rel > Para.v_max) {
            v_rel = Para.v_max;
        }
    }
    return v_rel;
}

void Spin::Stop() {
    stopFlag = true;

}
