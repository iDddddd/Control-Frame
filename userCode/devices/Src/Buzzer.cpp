//
// Created by 25396 on 2023/3/14.
//

#include "Buzzer.h"


bool buzzerWorkingFlag = false;

void bsp_BuzzerOn(float freq) {
    if (!buzzerWorkingFlag) {
        buzzerWorkingFlag = 1;
        HAL_TIM_Base_Start(&BUZZER_CLOCK);
        HAL_TIM_PWM_Start(&BUZZER_CLOCK, BUZZER_CLOCK_CHANNEL);
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, freq);
    }
}

void bsp_BuzzerOff() {
    if (buzzerWorkingFlag) {
        buzzerWorkingFlag = 0;
        HAL_TIM_PWM_Stop(&BUZZER_CLOCK, BUZZER_CLOCK_CHANNEL);
    }
}