//
// Created by 25396 on 2023/3/14.
//

#include "Buzzer.h"

uint16_t C_FREQ[]={0,262,294,330,349,392,440,
                   494,523,587,659,698,784,880,988,
                   1046,1175,1318,1397,1568,1760,1976};


uint8_t music_two_tiger_t_echo  = 150;
uint8_t P_2_1  = music_two_tiger_t_echo/2;
uint8_t P_1  = music_two_tiger_t_echo;
uint8_t P_4_1 =  music_two_tiger_t_echo/4;
uint8_t P_4_3  = music_two_tiger_t_echo/4*3;
uint8_t P_2  = music_two_tiger_t_echo*2;
uint8_t music_two_tiger_f[]={
        1,2,3,1,
        1,2,3,1,
        3,4,5,
        3,4,5,
        5,6,5,6,3,1,
        5,6,5,4,3,1,
        1,5,1,
        1,5,1,
};

uint16_t music_two_tiger_len=(sizeof(music_two_tiger_f)/sizeof(uint8_t));
bool buzzerWorkingFlag = false;


uint8_t music_two_tiger_t[]={
        P_1,P_1,P_1,P_1,
        P_1,P_1,P_1,P_1,
        P_1,P_1,P_2,
        P_1,P_1,P_2,
        P_4_3,P_4_1,P_4_3,P_4_1,P_1,P_1,
        P_4_3,P_4_1,P_4_3,P_4_1,P_1,P_1,
        P_1,P_1,P_2,
        P_1,P_1,P_2,
};

music_t two_tiger{
    .f = music_two_tiger_f,
    .t = music_two_tiger_t,
    .len = music_two_tiger_len,
    .t_each = music_two_tiger_t_echo,
    .now_len = 0
};



void passive_buzzer_set(uint16_t note_f)
{
    /*计算自动重装载值,计算新的频率*/
    uint16_t Autoreload=(80000.0/(float)C_FREQ[note_f])-1;
    /*计算音量*/
    uint16_t volume=(((float)Autoreload)/100.0f)*99.0f/*TEA_VOL*/;
    /*设置自动重装载值*/
    __HAL_TIM_SET_AUTORELOAD(&htim4,Autoreload);
    /*设置音量*/
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,volume);
    /*情况计算值*/
    __HAL_TIM_SET_COUNTER(&htim4,0);
}

uint32_t paly_delay_ms=0;
void my_buzzer_play()
{
    if(paly_delay_ms==0)
    {
        paly_delay_ms=two_tiger.t_each*two_tiger.t[two_tiger.now_len];
        passive_buzzer_set(two_tiger.f[two_tiger.now_len]);

        if(two_tiger.now_len>=(two_tiger.len-1))
        {
            bsp_BuzzerOff();
        }else
        {
            two_tiger.now_len++;
        }
    }else
    {
        paly_delay_ms--;
    }
}


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