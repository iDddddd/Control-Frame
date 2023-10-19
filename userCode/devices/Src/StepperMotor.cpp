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
    }else {
        if (STEP > 0) {
            STEP--;
        } else {
            HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
            CompleteTask(0x04);
            stopFlag = true;
        }
    }
}

void StepperMotor::Grab(bool _posflag) {

    stopFlag = false;
    if(posflag && !_posflag){
        DIR = 0;
        HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, GPIO_PIN_RESET);
        STEP = 1600;
        posflag = false;
    } else if(! posflag && _posflag){
        DIR = 1;
        HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin, GPIO_PIN_SET);
        STEP = 1600;
        posflag = true;
    }
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 499);
}
