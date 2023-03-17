//
// Created by 25396 on 2023/3/14.
//

#ifndef RM_FRAME_C_BUZZER_H
#define RM_FRAME_C_BUZZER_H
#include "Device.h"

#define BUZZER_CLOCK_FREQUENCY 84000000
#define BUZZER_CLOCK htim4
#define BUZZER_CLOCK_CHANNEL TIM_CHANNEL_3
#define REFERENCE_MAX_VOL_CCR 5000

typedef struct {
    uint8_t* f;
    uint8_t* t;
    uint16_t len;
    uint16_t t_each;
    uint16_t now_len;
}music_t;

void bsp_BuzzerOn(float freq);
void bsp_BuzzerOff();
void my_buzzer_play();


#endif //RM_FRAME_C_BUZZER_H
