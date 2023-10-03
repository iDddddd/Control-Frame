#include "StateEstimator.h"

State_t StateEstimator::Renew() {
    State_t retval = state;
    state = {0, 0, 0, state.vx, state.vy, state.w};
    return retval;
}

State_t StateEstimator::CurrentState() {
    return state;
}

/**
 * @brief sss
 * @param none
 * @todo: finish the function
*/
State_t StateEstimator::UpdateState(float* v8) {
    encoder_state = ReducEstimation(float* v8);
    //KalmanFilter();
    return encoder_state;
}

/**
 * @brief 执行8估3以及编码器状态估计
*/
State_t State_Estimator::ReducEstimation(float* v8) {
    //8估3
    State_t prev_state = encoder_state;
    State_t curr_state{ 0 };
    curr_state.vx = 0.25 * (v8[0] + v8[2] +v8[4] + v8[6]) * WHEEL_DIAMETER * PI / 360;
    curr_state.vy = 0.25 * (v8[1] + v8[3] +v8[5] + v8[7]) * WHEEL_DIAMETER * PI / 360;
    curr_state.w = 1.047 * (v8[0] - v8[1] + v8[2] + v8[3] - v8[4] + v8[5] - v8[6] + v8[7]) * WHEEL_DIAMETER * PI / 360;
    
    // 更新里程
    curr_state.theta = prev_state.theta + (curr_state.w + prev_state.w) * dT / 2;
    float curr_vxw = curr_state.vx * cos(curr_state.theta) + curr_state.vy * sin(curr_state.theta);// vxw代表世界坐标系的x速度
    float prev_vxw = prev_state.vx * cos(prev_state.theta) + prev_state.vy * sin(prev_state.theta);
    curr_state.x = prev_state.x + (curr_vxw + prev_vxw) * dT / 2;
    float curr_vyw = curr_state.vy * cos(curr_state.theta) - curr_state.vx * sin(curr_state.theta);
    float prev_vyw = prev_state.vy * cos(prev_state.theta) - prev_state.vx * sin(prev_state.theta);
    curr_state.y = prev_state.y + (curr_vyw + prev_vyw) * dT / 2;

    return curr_state;
}


State_t State_Estimator::KalmanFilter() {

}