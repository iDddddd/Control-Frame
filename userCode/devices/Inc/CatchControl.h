//
// Created by mac on 2022/12/14.
//

#ifndef RM_FRAME_C_CATCHCONTROL_H
#define RM_FRAME_C_CATCHCONTROL_H


#include "Device.h"

#define BUFF_SIZE 100

typedef struct {
    uint16_t speed;
    uint16_t angle;
} ARM_col_t;
typedef struct {
    ARM_col_t ARM1;
    ARM_col_t ARM2;
    ARM_col_t ARM3;
    uint16_t x;
    uint16_t y;
    uint8_t TrayFlag;
    uint8_t ChassisStopFlag;
    uint8_t ArmServoFlag;
} CC_ctrl_t;

class CatchControl : public Device {
public:
    static CC_ctrl_t cc_ctrl;
    static uint16_t data_length;
    static uint8_t rx_buff[2][BUFF_SIZE];


    static void Init();

    static void IT_Handle();

    static void GET_Data(const volatile uint8_t *buf);

};

/*外部函数声明-------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
extern void DMA2_Stream1_IRQHandler(void);
#ifdef __cplusplus
}
#endif

#endif //RM_FRAME_C_CATCHCONTROL_H
