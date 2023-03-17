//
// Created by David9686 on 2022/12/15.
//

#include "AutoMove.h"

float Move::expectPos[3]{};
uint8_t Move::FinishFlag = 0;

Move::Move() {
    a = 1.5;
    v_max = 2;
    pid.kp = 1;
    pid.ki = 0;
    pid.kd = 0;
};

Move::~Move() = default;

void Move::Calc(float target) {
    stopFlag = false;
    d_max = target;
    d1 = v_max * v_max / (2 * a);
    d2 = target - 2 * d1;
    if (d2 < 0) {
        d1 = target / 2;
        d2 = 0;
        v_max = sqrt(2 * a * d1);
    }


}


void Move::Stop() {
    stopFlag = true;
}

void Move::Handle(float &reference) {
    if (stopFlag) {
        v_rel = 0;
        reference = 0;
    } else {
        if (expectPos[Index] >= d_max) {
            v = 0;
        } else if (expectPos[Index] < d1) {
            v += a * 0.001f;
            expectPos[Index] += (2 * v - a * 0.001f) / 2 * 0.001f;
        } else if (expectPos[Index] > (d1 + d2)) {
            v -= a * 0.001f;
            expectPos[Index] += (2 * v + a * 0.001f) / 2 * 0.001f;
        } else if (expectPos[Index] > d1 && expectPos[Index] < (d1 + d2)) {
            expectPos[Index] += v * 0.001f;
        }

        if (reference < expectPos[Index]) {
            v_rel = v + 0.05f * a;
        } else if (reference > expectPos[Index]) {
            v_rel = v - 0.05f * a;
        } else if (v < 0) {
            v_rel = 0;
        } else {
            v_rel = v;
        }
      //  v_rel = v + pid.PIDCalc(expectPos[Index], reference, 1);
        if (v_rel <= 0) {
            stopFlag = true;
            FinishFlag += 1;
        }
    }
}

AutoMove::AutoMove() {
    x.Index = 0;
    y.Index = 1;
    o.Index = 2;
}

void AutoMove::Handle() {
    if (StopFlag) {
        StopMove();
    } else {
        x.Handle(IMU::imu.position.displace[1]);
        y.Handle(IMU::imu.position.displace[0]);
        o.Handle(IMU::imu.attitude.yaw);
    }
    /*  if(Move::FinishFlag == 3){
          uint8_t flag = 0x01;
          HAL_UART_Transmit_IT(&huart6,&flag,1);
          StopFlag = true;
      }//完成后发送*/
}

void AutoMove::StartMove(float x_distance, float y_distance, float o_angle) {
    StopFlag = false;
    x.Calc(x_distance);
    y.Calc(y_distance);
    o.Calc(o_angle);
    IMU::imu.position.displace[0] = 0;
    IMU::imu.position.displace[1] = 0;
    Move::FinishFlag = 0;
}

void AutoMove::StopMove() {
    StopFlag = true;
    x.Stop();
    y.Stop();
    o.Stop();
}
