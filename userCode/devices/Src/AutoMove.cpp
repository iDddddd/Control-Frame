//
// Created by David9686 on 2022/12/15.
//

#include "AutoMove.h"

float Move::t = 0.0;
float Move::v = 0;
float Move::x = 0;
float Move::a = 2.0;
float Move::v_max = 1.0;


Move::Move(float d){
    d_max = d;
    d1 = v_max * v_max / (2 * a);
    d2 = d - 2 * d1;
    if (d2 < 0){
        d1 = d / 2;
    }
}
void Move::Handle(){
    t += 0.001;

    if (d2 >= 0) {
        if (x < d1) {
            v += a * 0.001;
            x += (2 * v - a * 0.001) / 2 * 0.001;
        }
        else if (x > (d1 + d2)) {
            v -= a * 0.001;
            x += (2 * v + a * 0.001) / 2 * 0.001;
        }
        else if (x > d1 && x < (d1 + d2)){
            x += v*0.001;
        }
        else if(x >= d_max){
            v = 0 ;
        }

        if(IMU::imu.position.displace[1] < x){
            v_rel = v + 0.001 * a;
        }
        else if (IMU::imu.position.displace[1] > x){
            v_rel = v - 0.001 * a;
        }
        else if (v < 0){
            v_rel = 0;
        }
        else{
            v_rel = v;
        }
        if (v_rel > v_max){
            v_rel = v_max;
        }

    }
    else if (d2 < 0){
        v_rel = 0;
    }

}
Move::~Move(){

}