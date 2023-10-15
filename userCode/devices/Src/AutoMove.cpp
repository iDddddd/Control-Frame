//
// Created by David9686 on 2022/12/15.
//

#include "AutoMove.h"

Move_X X;
Move_Y Y;
Spin O;
float tem_vx,tem_vy;
extern float encoder_x, encoder_y, encoder_theta;

void AutoMove::Handle() {
    /*if (!StopFlag)*/ {
        // vx = X.Handle(IMU::imu.position.displace[1]);
        // vy = Y.Handle(IMU::imu.position.displace[0]);
        // vo = O.Handle(IMU::imu.attitude.yaw);
        vx = X.Handle(encoder_x);
        vy = Y.Handle(encoder_y);
        vo = O.Handle(encoder_theta);
        //vo = 0;//测试用，完了记得删
        
        tem_vx = vx * cos(encoder_theta) - vy * sin(encoder_theta);
        tem_vy = vy * cos(encoder_theta) + vx * sin(encoder_theta);
        vx = tem_vx;
        vy = tem_vy;
    } /*else {
        AutoChassisStop();
    }*/
    if (X.FinishFlag && Y.FinishFlag && O.FinishFlag && !SendFlag) {
        StopMove();
        CompleteTask(0x02);//?
        SendFlag = true;
    }//完成后发送
}

void AutoMove::StartMove(float x_distance, float y_distance, float o_angle) {
    StopFlag = false;
    SendFlag = false;
    X.Calc(x_distance);
    Y.Calc(y_distance);
    O.Calc(o_angle);
    encoder_theta = 0;
    encoder_x = 0;
    encoder_y = 0;
    IMU::imu.position.displace[0] = 0;
    IMU::imu.position.displace[1] = 0;
}

void AutoMove::StopMove() {
    StopFlag = true;
    X.Stop();
    Y.Stop();
    O.Stop();
    // WheelsSpeedCalc(0, 0, 0);//强行把底盘速度置0，不然编码器速度不会归零，目前不太清楚为什么
}


Move_X::Move_X() {
    Para.a = 1.5;
    Para.v_max = 2;
    pid.kp = 1;
    pid.ki = 0;
    pid.kd = 0;
}

void Move_X::Calc(float target) {
    if(target == 0) {
        FinishFlag = true;
        ReachFlag = true;
        stopFlag = false;
    }
    else if(target > 0) {
        Para.a = 1.5;
        Para.v_max = 2;
        stopFlag = false;
        FinishFlag = false;
        ReachFlag = false;
        finishcount = 0;
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
    else {
        Para.a = -1.5;
        Para.v_max = -2;
        stopFlag = false;
        FinishFlag = false;
        ReachFlag = false;
        finishcount = 0;
        expectPos = 0;
        Para.d_max = target;
        Para.d1 = Para.v_max * Para.v_max / (2 * Para.a);
        Para.d2 = target - 2 * Para.d1;
        if (Para.d2 > 0) {
            Para.d1 = target / 2;
            Para.d2 = 0;
            Para.v_max = -sqrt(2 * Para.a * Para.d1);
        }
    }
}

float Move_X::Handle(float reference) {
    if (stopFlag) {
        v_rel = 0;
    }
    else {// 估速度输出
        if (abs(expectPos) >= abs(Para.d_max)) {
            Para.v = 0;
        } else if (abs(expectPos) < abs(Para.d1)) {
            Para.v += Para.a * 0.001f;
            expectPos += (2 * Para.v - Para.a * 0.001f) / 2 * 0.001f;
        } else if (abs(expectPos) > abs(Para.d1 + Para.d2)) {
            Para.v -= Para.a * 0.001f;
            expectPos += (2 * Para.v + Para.a * 0.001f) / 2 * 0.001f;
        } else if (abs(expectPos) > abs(Para.d1) && abs(expectPos) < abs(Para.d1 + Para.d2)) {
            expectPos += Para.v * 0.001f;
        }

        v_rel = Para.v + pid.PIDCalc(expectPos, reference, 2.0);
        if (abs(v_rel) > abs(Para.v_max) && Para.v_max != 0) {
            v_rel = Para.v_max;
        }
    }

    if (abs(Para.d_max - reference) <= 0.01) {
        ReachFlag = true;
    }
    if(ReachFlag) {
        pid.kp = 5;
        if(finishcount < 500) finishcount++;
        if(finishcount == 500) {
            FinishFlag = true;
        }
    }

    if(abs(v_rel) < 0.02) {
        v_rel = 0;
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
    if(target == 0) {
        FinishFlag = true;
        ReachFlag = true;
        stopFlag = false;
    }
    else if (target > 0) {
        Para.a = 1.5;
        Para.v_max = 2;
        stopFlag = false;
        FinishFlag = false;
        ReachFlag = false;
        finishcount = 0;
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
    else {
        Para.a = -1.5;
        Para.v_max = -2;
        stopFlag = false;
        FinishFlag = false;
        ReachFlag = false;
        finishcount = 0;
        expectPos = 0;
        Para.d_max = target;
        Para.d1 = Para.v_max * Para.v_max / (2 * Para.a);
        Para.d2 = target - 2 * Para.d1;
        if (Para.d2 > 0) {
            Para.d1 = target / 2;
            Para.d2 = 0;
            Para.v_max = -sqrt(2 * Para.a * Para.d1);
        }

    }
}

float Move_Y::Handle(float reference) {
    if (stopFlag) {
        v_rel = 0;
    }
    else {// 估速度输出
        if (abs(expectPos) >= abs(Para.d_max)) {
            Para.v = 0;
        } else if (abs(expectPos) < abs(Para.d1)) {
            Para.v += Para.a * 0.001f;
            expectPos += (2 * Para.v - Para.a * 0.001f) / 2 * 0.001f;
        } else if (abs(expectPos) > abs(Para.d1 + Para.d2)) {
            Para.v -= Para.a * 0.001f;
            expectPos += (2 * Para.v + Para.a * 0.001f) / 2 * 0.001f;
        } else if (abs(expectPos) > abs(Para.d1) && abs(expectPos) < abs(Para.d1 + Para.d2)) {
            expectPos += Para.v * 0.001f;
        }

        v_rel = Para.v + pid.PIDCalc(expectPos, reference, 2.0);
        if (abs(v_rel) > abs(Para.v_max) && Para.v_max != 0) {
            v_rel = Para.v_max;
        }
    }
        
    if (abs(Para.d_max - reference) <= 0.01) {
        ReachFlag = true;
    }

    if(ReachFlag) {
        pid.kp = 5;
        if(finishcount < 500) finishcount++;
        if(finishcount == 500) {
            FinishFlag = true;
        }
    }

    if(abs(v_rel) < 0.02) {
        v_rel = 0;
    }

    return v_rel;
}

void Move_Y::Stop() {
    stopFlag = true;

}

Spin::Spin() {
    Para.a = 10;
    Para.v_max = 8;
    pid.kp = 1.6;// 原2.2
    pid.ki = 0;
    pid.kd = 0;

}

void Spin::Calc(float target) {
    pid.kp = 1.85 + 1.25 * (PI - target) / PI;
    pid.ki = 0;
    if (abs(target) < 0.2 && target != 0) {
        pid.ki = 0.006;
        // pid.kp = 50;
    }
    if(target == 0) {// 纠偏
        FinishFlag = true;
        stopFlag = false;
        pid.kp = 6; // 先试试吧……
        Para.a = 0;
        Para.v_max = 0;
        expectPos = 0;
        Para.d_max = Para.d1 = Para.d2 = 0;
    }
    else if(target > 0) {
        Para.a = 10;
        Para.v_max = 8;
        stopFlag = false;
        FinishFlag = false;
        ReachFlag = false;
        finishcount = 0;
        expectPos = 0;
        // Para.d_max = target-IMU::imu.attitude.yaw;
        Para.d_max = target;
        Para.d1 = Para.v_max * Para.v_max / (2 * Para.a);
        Para.d2 = target - 2 * Para.d1;
        if (Para.d2 < 0) {
            Para.d1 = target / 2;
            Para.d2 = 0;
            Para.v_max = sqrt(2 * Para.a * Para.d1);
        }
    }
    else{
        Para.a = -10;
        Para.v_max = -8;
        stopFlag = false;
        FinishFlag = false;
        ReachFlag = false;
        finishcount = 0;
        Para.d_max = target;
        Para.d1 = Para.v_max * Para.v_max / (2 * Para.a);
        Para.d2 = target - 2 * Para.d1;
        if(Para.d2 > 0){
            Para.d1 = target / 2;
            Para.d2 = 0;
            Para.v_max = -sqrt(2 * Para.a * Para.d1);
        }
    }
    
}

float Spin::Handle(const float reference) {
    if (stopFlag) {
        v_rel = 0;
    }
    else {
        if (abs(expectPos) >= abs(Para.d_max)) {
            Para.v = 0;
        } else if (abs(expectPos) < abs(Para.d1)) {
            Para.v += Para.a * 0.001f;
            expectPos += (2 * Para.v - Para.a * 0.001f) / 2 * 0.001f;
        } else if (abs(expectPos) > abs(Para.d1 + Para.d2)) {
            Para.v -= Para.a * 0.001f;
            expectPos += (2 * Para.v + Para.a * 0.001f) / 2 * 0.001f;
        } else if (abs(expectPos) > abs(Para.d1) && abs(expectPos) < abs(Para.d1 + Para.d2)) {
            expectPos += Para.v * 0.001f;
        }

        v_rel = Para.v + pid.PIDCalc(expectPos, reference, 12);
        if (abs(v_rel) > abs(Para.v_max) && Para.v_max != 0) {  // 要区分对待纠偏和正常转
            v_rel = Para.v_max;
        }
    }

    if ((abs(reference - Para.d_max) < 0.01 * abs(Para.d_max))){
        ReachFlag = true;
        FinishFlag = true;
    }
    if(ReachFlag) {
        //pid.kp = 300;
        if(finishcount < 500) finishcount++;
        if(finishcount == 500) {
            FinishFlag = true;
        }
    }
    return v_rel;
}

void Spin::Stop() {
    stopFlag = true;
}
