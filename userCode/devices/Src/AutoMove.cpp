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

void Move::Handle_X() {
    static float x = 0;
    if (x < d1) {
        v += a * 0.001f;
        x += (2.0f * v - a * 0.001f) / 2 * 0.001f;
    } else if (x > (d1 + d2)) {
        v -= a * 0.001f;
        x += (2.0f * v + a * 0.001f) / 2 * 0.001f;
    } else if (x > d1 && x < (d1 + d2)) {
        x += v * 0.001f;
    } else if (x >= d_max) {
        v = 0;
    }

    if (IMU::imu.position.displace[1] < x) {
        v_rel = v + 0.001f * a;
    } else if (IMU::imu.position.displace[1] > x) {
        v_rel = v - 0.001f * a;
    } else if (v < 0) {
        v_rel = 0;
    } else {
        v_rel = v;
    }

    if (v_rel > v_max) {
        v_rel = v_max;
    } else if (v_rel < 0) {
        v_rel = 0;
        a = 0;
    }


}

void Move::Handle_Y() {
    static float y = 0;
    if (y < d1) {
        v += a * 0.001f;
        y += (2 * v - a * 0.001f) / 2 * 0.001f;
    } else if (y > (d1 + d2)) {
        v -= a * 0.001f;
        y += (2 * v + a * 0.001f) / 2 * 0.001f;
    } else if (y > d1 && y < (d1 + d2)) {
        y += v * 0.001f;
    } else if (y >= d_max) {
        v = 0;
    }

    if (IMU::imu.position.displace[0] < y) {
        v_rel = v + 0.001f * a;
    } else if (IMU::imu.position.displace[0] > y) {
        v_rel = v - 0.001f * a;
    } else if (v < 0) {
        v_rel = 0;
    } else {
        v_rel = v;
    }

    if (v_rel > v_max) {
        v_rel = v_max;
    } else if (v_rel < 0) {
        v_rel = 0;
        a = 0;
    }

}

void Move::Handle_O() {
    static float o = 0;
    if (o < d1) {
        v += a * 0.001f;
        o += (2 * v - a * 0.001f) / 2 * 0.001f;
    } else if (o > (d1 + d2)) {
        v -= a * 0.001f;
        o += (2 * v + a * 0.001f) / 2 * 0.001f;
    } else if (o > d1 && o < (d1 + d2)) {
        o += v * 0.001f;
    } else if (o >= d_max) {
        v = 0;
    }

    if (IMU::imu.attitude.yaw < o) {
        v_rel = v + 0.001f * a;
    } else if (IMU::imu.attitude.yaw > o) {
        v_rel = v - 0.001f * a;
    } else if (v < 0) {
        v_rel = 0;
    } else {
        v_rel = v;
    }
    if (v_rel > v_max) {
        v_rel = v_max;
    } else if (v_rel < 0) {
        v_rel = 0;
        a = 0;
    }

}

AutoMove::AutoMove(float x_distance, float y_distance, float o_angle) {
    x.Calc(x_distance);
    y.Calc(y_distance);
    o.Calc(o_angle);
}

void AutoMove::Handle() {
    x.Handle_X();
    y.Handle_Y();
    o.Handle_O();
}
