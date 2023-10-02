//
// Created by 25396 on 2023/9/7.
//

#include "StepperMotor.h"


StepperMotor::~StepperMotor() = default;

StepperMotor::StepperMotor(MOTOR_INIT_t *_init) : Motor(_init, this) {

}

void StepperMotor::Handle() {
    if (stopFlag) {
        HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
    }
    if(STEP > 0){
        STEP--;
    } else{
        HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
    }
}

void StepperMotor::Grab(bool _posflag) {

    stopFlag = false;
    if(posflag && !_posflag){
        DIR = 0;
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_RESET);
        STEP = 1000;
        posflag = false;
    } else if(!posflag && _posflag){
        DIR = 1;
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_SET);
        STEP = 1000;
        posflag = true;
    }
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 499);
}
