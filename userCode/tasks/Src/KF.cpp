#include "KF.h"
//变量定义
float prev_encoder_theta = 0.0;
float encoder_theta = 0.0;
float encoder_x = 0.0;
float encoder_y = 0.0;//编码器里程估计值
extern float v1x, v1y, v2x, v2y, v3x, v3y, v4x, v4y;//单轮数据

float T83[24] = { 0.25,0,0.25,0,0.25,0,0.25,0,
	0,0.25,0,0.25,0,0.25,0,0.25,
	1.0417,-1.0417,1.0417,1.0417,-1.0417,1.0417,-1.0417,-1.0417 };//8估3矩阵

float F55[25] = { 1,0,dT,0,0,  0,1,0,dT,0,  0,0,1,0,0,  0,0,0,1,0,  0,0,0,0,1 };

float I55[25] = { 1,0,0,0,0,  0,1,0,0,0,  0,0,1,0,0,  0,0,0,1,0,  0,0,0,0,1 };
float I33[9] = { 1,0,0,  0,1,0,  0,0,1 };


//建立矩阵
Matrix encoder_v3(3, 1);
Matrix prev_encoder_v3(3, 1);
Matrix prev_u(3,1);
const Matrix trans8_3(T83, 3, 8);
const Matrix F(F55, 5, 5);
Matrix z(3, 1);//观测量，初始化为全0
Matrix state(5, 1);//状态量，初始化为全0，分别为：x、y、vx、vy、theta
Matrix IMUstate(5,1);
Matrix P(I55, 5, 5);
Matrix K(5, 3);
const Matrix Q(I55, 5, 5);
const Matrix R(I33, 3, 3);
const Matrix I(I55, 5, 5);

//函数定义
void convert8_3() {
    //更新prev
    prev_encoder_v3 = encoder_v3;

    //编码器数据读入
    float v8[8] = { -v1x, -v1y, v2x, v2y, v3x, v3y, -v4x, -v4y };//此处完成取反
    Matrix V8(v8, 8, 1);

    //8估3
    encoder_v3 = trans8_3 * V8;
    
    //速度修正
    encoder_v3.data[0][0] = encoder_v3.data[0][0] * WHEEL_DIAMETER * PI / 360;
    encoder_v3.data[1][0] = encoder_v3.data[1][0] * WHEEL_DIAMETER * PI / 360;
    encoder_v3.data[2][0] = encoder_v3.data[2][0] * WHEEL_DIAMETER * PI / 360;//我也不知道为什么是360；理论上应该是180的
    //观测量更新
    z.data[0][0] = encoder_v3.data[0][0];
    z.data[1][0] = encoder_v3.data[1][0];
    z.data[2][0] += encoder_v3.data[2][0] * dT;
}


void get_encoder_mileage(){
    prev_encoder_theta = encoder_theta;
    encoder_theta += (encoder_v3.data[2][0] + prev_encoder_v3.data[2][0]) * dT / 2;
    encoder_x += (encoder_v3.data[0][0] * cos(encoder_theta) + encoder_v3.data[1][0] * sin(encoder_theta) + prev_encoder_v3.data[0][0] * cos(prev_encoder_theta) + prev_encoder_v3.data[1][0] * sin(prev_encoder_theta)) / 2 * dT;
    encoder_y += (-encoder_v3.data[0][0] * sin(encoder_theta) + encoder_v3.data[1][0] * cos(encoder_theta) - prev_encoder_v3.data[0][0] * sin(prev_encoder_theta) + prev_encoder_v3.data[1][0] * cos(prev_encoder_theta)) / 2 * dT;
}

/**/
void KalmanFilter() {
    //IMU数据读入
    float u3[3] = { IMU::imu.position._accel[0],IMU::imu.position._accel[1],IMU::imu.attitude.yaw_v };
    Matrix u(u3, 3, 1);
    //滤波，如果加速度太小，则视为加速度计的漂移
    if (abs(u3[0]) <= 0.0055) {
        IMUstate.data[2][0] = 0.0;
        state.data[2][0] = 0.0;
        u3[0] = 0.0;
    }
    if (abs(u3[1]) <= 0.0055) {
        IMUstate.data[3][0] = 0.0;
        state.data[3][0] = 0.0;
        u3[1] = 0.0;
    }
    //纯IMU预测
    float IMUyaw = IMUstate.data[4][0];
    float IMU_B53[15] = { dT * dT * cos(IMUyaw) / 2,dT * dT * sin(IMUyaw) / 2,0,
        -dT * dT * sin(IMUyaw) / 2,dT * dT * cos(IMUyaw) / 2,0,
        dT * cos(IMUyaw),dT * sin(IMUyaw),0,
        -dT * sin(IMUyaw),dT * cos(IMUyaw),0,
        0,0,dT };
    Matrix IMU_B(IMU_B53,5,3);
    IMUstate = F * IMUstate + IMU_B * u;
    
    //预测
    float theta = state.data[4][0];
    float B53[15] = { dT * dT * cos(theta) / 2,dT * dT * sin(theta) / 2,0,
        -dT * dT * sin(theta) / 2,dT * dT * cos(theta) / 2,0,
        dT * cos(theta),dT * sin(theta),0,
        -dT * sin(theta),dT * cos(theta),0,
        0,0,dT };
    Matrix B(B53, 5, 3);
    state = F * state + B * u;
    //预测P
    P = F * P * F.transpose() + Q;


    //更新
    float H35[15] = { 0,0,cos(theta),sin(theta),0,
        0,0,-sin(theta),cos(theta),0,
        0,0,0,0,1 };
    Matrix H(H35, 3, 5);
    //卡尔曼增益
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    //获取状态最优估计
    state = state + K * (z - H * state);
    //更新协方差矩阵
    P = (I - K * H) * P;

    prev_u = u;
}


void set_mileage_zero() {
    encoder_theta = prev_encoder_theta = 0.0;
    encoder_x = encoder_y = 0.0;
}


void KalmanFilter2() {
    //IMU数据读入
    float u3[3] = { IMU::imu.position._accel[0],IMU::imu.position._accel[1],IMU::imu.attitude.yaw };
    Matrix u(u3, 3, 1);
    //滤波，如果加速度太小，则视为加速度计的漂移
    if (abs(u3[0]) <= 0.0055) {
        IMUstate.data[2][0] = 0.0;
        state.data[2][0] = 0.0;
        u3[0] = 0.0;
    }
    if (abs(u3[1]) <= 0.0055) {
        IMUstate.data[3][0] = 0.0;
        state.data[3][0] = 0.0;
        u3[1] = 0.0;
    }
    //纯IMU预测
    float IMUyaw = IMUstate.data[4][0];
    float IMU_B53[15] = { dT * dT * cos(IMUyaw) / 2,dT * dT * sin(IMUyaw) / 2,0,
        -dT * dT * sin(IMUyaw) / 2,dT * dT * cos(IMUyaw) / 2,0,
        dT * cos(IMUyaw),dT * sin(IMUyaw),0,
        -dT * sin(IMUyaw),dT * cos(IMUyaw),0,
        0,0,1 };
    Matrix IMU_B(IMU_B53,5,3);
    IMUstate = F * IMUstate + IMU_B * u;
    
    //预测
    float theta = state.data[4][0];
    float B53[15] = { dT * dT * cos(theta) / 2,dT * dT * sin(theta) / 2,0,
        -dT * dT * sin(theta) / 2,dT * dT * cos(theta) / 2,0,
        dT * cos(theta),dT * sin(theta),0,
        -dT * sin(theta),dT * cos(theta),0,
        0,0,1 };
    Matrix B(B53, 5, 3);
    state = F * state + B * u;
    //预测P
    P = F * P * F.transpose() + Q;


    //更新
    float H35[15] = { 0,0,cos(theta),sin(theta),0,
        0,0,-sin(theta),cos(theta),0,
        0,0,0,0,1 };
    Matrix H(H35, 3, 5);
    //卡尔曼增益
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    //获取状态最优估计
    state = state + K * (z - H * state);
    //更新协方差矩阵
    P = (I - K * H) * P;

    prev_u = u;
}