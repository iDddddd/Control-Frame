#pragma once

#include "IMU.h"
#include "ChassisTask.h"
#include "Matrix.h"
#include "math.h"


const float dT = 0.001;

//float encoder_vx, encoder_vy, encoder_w;




//º¯Êý¶¨Òå
void convert8_3();
void get_encoder_mileage();
void KalmanFilter();
void KalmanFilter2();