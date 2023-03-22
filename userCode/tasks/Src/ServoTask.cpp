//
// Created by David9686 on 2022/12/16.
//

#include "ServoTask.h"


SERVO_INIT_T ArmServo = {
        .servoID = SERVO_ID_1,
        .firstAngle = 900,
        .angleLimit_Min = 0,
        .angleLimit_Max = 2200,
};

SERVO_INIT_T TrayServo = {
        .servoID = SERVO_ID_2,
        .firstAngle = 930,
        .angleLimit_Min = 800,
        .angleLimit_Max = 2200,
};

Servo armServo(&ArmServo);
Servo trayServo(&TrayServo);
/*void AutoTraySet(uint8_t trayflag){
    if(trayflag == 1){
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
                              930);
    }else{
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
                              900);
    }
    uint8_t flag = 0x01;
    HAL_UART_Transmit_IT(&huart6,&flag,1);
}*/
void AutoClawSet(uint8_t clawflag){
    if(clawflag == 1){
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                              1550);
    }else{
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                              1300);
    }
    uint8_t flag = 0x01;
    HAL_UART_Transmit(&huart6,&flag,1,3);
}


void ServoHandle() {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                          1300);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
                          930);
}


