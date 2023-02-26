//
// Created by David9686 on 2022/12/15.
//

#include "AutoMove.h"

Move::Move() {

}

Move::~Move() = default;

void Move::Calc(float target) {
    d_max = target;
    d1 = v_max * v_max / (2 * a);
    d2 = target - 2 * d1;
    if (d2 < 0) {
        d1 = target / 2;
        d2 = 0;
    }
}

void Move::Handle() {
    static float x = 0;
    if (x < d1) {
        v += a * 0.001;
        x += (2 * v - a * 0.001) / 2 * 0.001;
    } else if (x > (d1 + d2)) {
        v -= a * 0.001;
        x += (2 * v + a * 0.001) / 2 * 0.001;
    } else if (x > d1 && x < (d1 + d2)) {
        x += v * 0.001;
    } else if (x >= d_max) {
        v = 0;
    }

    if (IMU::imu.position.displace[1] < x) {
        v_rel = v + 0.001 * a;
    } else if (IMU::imu.position.displace[1] > x) {
        v_rel = v - 0.001 * a;
    } else if (v < 0) {
        v_rel = 0;
    } else {
        v_rel = v;
    }
    if (v_rel > v_max) {
        v_rel = v_max;
    }

}

AutoMove::AutoMove(float x_distance, float y_distance, float o_angle) {
    x.Calc(x_distance);
    y.Calc(y_distance);
    o.Calc(o_angle);
}

void AutoMove::Handle() {

}
