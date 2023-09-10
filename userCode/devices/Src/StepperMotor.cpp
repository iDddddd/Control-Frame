//
// Created by 25396 on 2023/9/7.
//

#include "StepperMotor.h"


StepperMotor::~StepperMotor() = default;

StepperMotor::StepperMotor(MOTOR_INIT_t *_init) : Motor(_init, this) {

}

void StepperMotor::Handle() {
    static int step_count = 0;
    if (stopFlag) {
        STEP = 0;
    }
    step_count++;
    if(step_count>1) {
        if (STEP > 0) {
            HAL_GPIO_TogglePin(STEP_GPIO_Port, STEP_Pin);
            STEP--;
        }
        step_count = 0;
    }
}

void StepperMotor::Grab(bool _posflag) {

    stopFlag = false;
    if(posflag && !_posflag){
        DIR = 0;
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
        STEP = 300;
        posflag = false;
    } else if(!posflag && _posflag){
        DIR = 1;
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
        STEP = 300;
        posflag = true;
    }
}
