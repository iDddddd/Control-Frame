#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include "Matrix.h"

const float dT = 0.001f;

typedef struct {
    float x;
    float y;
    float theta;
    float vx;
    float vy;
    float w;
} State_t;


class StateEstimator {
public:
    State_t Renew();
    State_t CurrentState();
    State_t UpdateState(float* v8);
private:
    State_t ReducEstimation(float* v8);
    State_t KalmanFilter();

    State_t state{ 0 };
    State_t imu_state{ 0 };
    State_t encoder_state{ 0 };
}

#endif //STATE_ESTIMATOR_H