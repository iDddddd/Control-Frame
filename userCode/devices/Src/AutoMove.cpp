#include "AutoMove.h"

AutoMove automove;


extern Chassis chassis;

void AutoMove::Handle() {    
    autosetvelocity.vx = X.Handle(chassis.getOdom().x);
    autosetvelocity.vy = Y.Handle(chassis.getOdom().y);
    autosetvelocity.w = O.Handle(chassis.getOdom().theta);    
    // 世界系->车身系
    float tem_vx,tem_vy;
    tem_vx = autosetvelocity.vx * cos(chassis.getOdom().theta) - autosetvelocity.vy * sin(chassis.getOdom().theta);
    tem_vy = autosetvelocity.vy * cos(chassis.getOdom().theta) + autosetvelocity.vx * sin(chassis.getOdom().theta);
    autosetvelocity.vx = tem_vx;
    autosetvelocity.vy = tem_vy;

    if (X.FinishFlag && Y.FinishFlag && O.FinishFlag && !SendFlag) {
        // StopMove();
        CompleteTask();
        SendFlag = true;
    }//完成后发送
}

void AutoMove::StartMove(float x_distance, float y_distance, float o_angle) {
    StopFlag = false;
    SendFlag = false;
    X.Calc(x_distance);
    Y.Calc(y_distance);
    O.Calc(o_angle);

    chassis.OdomReset();
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
    expectPos = 0;
    Para.d_max = target;
    if (target == 0) {
        FinishFlag = true;        
        Para.a = 0;
        Para.v_max = 0;
    }
    else {
        FinishFlag = false;
        Para.a = (target > 0) ? 1.5 : -1.5;
        Para.v_max = (target > 0) ? 2 : -2;        
        Para.d1 = Para.v_max * Para.v_max / (2 * Para.a);
        Para.d2 = target - 2 * Para.d1;
        if (Para.d2 * target < 0) { // 目标为正但d2为负，或目标为负d2为正
            Para.d1 = target / 2;
            Para.d2 = 0;
            Para.v_max = (target > 0) ? sqrt(2 * Para.a * Para.d1) : -sqrt(2 * Para.a * Para.d1);
        }
    }
}

float Move_X::Handle(float reference) {
    if (stopFlag) {
        v_rel = 0;
    }
    else {
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

    if (abs(Para.d_max - reference) <= 0.01 * abs(Para.d_max)) {
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
    expectPos = 0;
    Para.d_max = target;
    if (target == 0) {
        FinishFlag = true;
        Para.a = 0;
        Para.v_max = 0;
    }
    else {
        FinishFlag = false;
        Para.a = (target > 0) ? 1.5 : -1.5;
        Para.v_max = (target > 0) ? 2 : -2;
        Para.d1 = Para.v_max * Para.v_max / (2 * Para.a);
        Para.d2 = target - 2 * Para.d1;
        if (Para.d2 * target < 0) { // 目标为正但d2为负，或目标为负d2为正
            Para.d1 = target / 2;
            Para.d2 = 0;
            Para.v_max = (target > 0) ? sqrt(2 * Para.a * Para.d1) : -sqrt(2 * Para.a * Para.d1);
        }
    }
}

float Move_Y::Handle(float reference) {
    if (stopFlag) {
        v_rel = 0;
    }
    else {
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
    if (abs(Para.d_max - reference) <= 0.01 * abs(Para.d_max)) {
        FinishFlag = true;
    }
    return v_rel;
}

void Move_Y::Stop() {
    stopFlag = true;
}

Spin::Spin() {
    Para.a = 10;
    Para.v_max = 8;
    pid.kp = 1.6;
    pid.ki = 0;
    pid.kd = 0;

}

void Spin::Calc(float target) {
    stopFlag = false;
    expectPos = 0;
    Para.d_max = target;
    if (target == 0) {
        FinishFlag = true;        
        Para.a = 0;
        Para.v_max = 0;
    }
    else {
        FinishFlag = false;
        Para.a = (target > 0) ? 10 : -10;
        Para.v_max = (target > 0) ? 8 : -8;
        //Para.d_max = target-IMU::imu.attitude.yaw;
        Para.d_max = target;
        Para.d1 = Para.v_max * Para.v_max / (2 * Para.a);
        Para.d2 = target - 2 * Para.d1;
        if (Para.d2 * target < 0) {
            Para.d1 = target / 2;
            Para.d2 = 0;
            Para.v_max = (target > 0) ? sqrt(2 * Para.a * Para.d1) : -sqrt(2 * Para.a * Para.d1);
        }
    }
}

float Spin::Handle(const float reference) {
    if (stopFlag) {
        v_rel = 0;
    }
    else {
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

        v_rel = Para.v + pid.PIDCalc(expectPos, reference, 0.5);
        if (v_rel > Para.v_max) {
            v_rel = Para.v_max;
        }
    }
    if (abs(reference - Para.d_max) < 0.01 * abs(Para.d_max)) {
        FinishFlag = true;
    }
    return v_rel;
}

void Spin::Stop() {
    stopFlag = true;
}
