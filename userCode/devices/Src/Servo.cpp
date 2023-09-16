//
// Created by LEGION on 2021/10/5.
//

#include "Servo.h"
/*
const SERVO_HANDLE_T servoInfo[4]={
        {&htim1, TIM_CHANNEL_1},
        {&htim1, TIM_CHANNEL_2},
        {&htim1, TIM_CHANNEL_3},
        {&htim1, TIM_CHANNEL_4},
};
uint32_t Servo::servo_IDs;
*//**
 * @brief 舵机类的初始化函数，开启了相关定时器
 *//*
void Servo::Init() {
    HAL_TIM_Base_Start_IT(&htim1);
}
*//**
 * @brief 舵机的构造函数，进行舵机的初始化设置
 * @param servoInit 舵机的初始化结构体指针
 *//*
Servo::Servo(SERVO_INIT_T *servoInit) {
    deviceID = servoInit->servoID;
    deviceType = SERVO;

    angleLimit_Min = servoInit->angleLimit_Min;
    angleLimit_Max = servoInit->angleLimit_Max;

    servo_IDs |= deviceID;
}

void Servo::stop() {
    if (stopFlag == 0){
        stopFlag = 1;
        auto pos = deviceID;
        HAL_TIM_PWM_Stop(servoInfo[pos].handleTypeDef,servoInfo[pos].timChannel);
    }
}
*//**
 * @brief 设置舵机角度
 * @param _targetAngle
 *//*
void Servo::SetTargetAngle(float _targetAngle) {
    if(stopFlag == 1){
        stopFlag = 0;
        auto pos = deviceID;

        HAL_TIM_PWM_Start(servoInfo[pos].handleTypeDef,servoInfo[pos].timChannel);
    }
    INRANGE(_targetAngle,angleLimit_Min,angleLimit_Max);
    targetAngle = _targetAngle;
}
*//**
 * @brief 舵机的控制驱动函数
 *//*
void Servo::Handle() {
        duty = targetAngle;
        auto pos = deviceID;

        __HAL_TIM_SET_COMPARE(servoInfo[pos].handleTypeDef,servoInfo[pos].timChannel,
                              htim1.Instance->ARR*duty);
}

void Servo::ErrorHandle() {

}*/

