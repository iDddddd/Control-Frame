//
// Created by David9686 on 2022/12/10.
//

#ifndef RM_FRAME_C_CATCHCONTROL_H
#define RM_FRAME_C_CATCHCONTROL_H

#include "Device.h"

#define BUFF_SIZE 100

typedef struct {
    uint16_t velocity;
    uint16_t angle;
}ARM_col_t;
typedef struct {
    ARM_col_t ARM1;
    ARM_col_t ARM2;
    ARM_col_t ARM3;
    uint8_t TrayFlag;
}CC_ctrl_t;


class CatchControl : public Device {
public:
    static CC_ctrl_t cc_ctrl;
    static uint8_t data_length;
    static uint8_t rx_buff[BUFF_SIZE];


    static void Init();
    static void IT_Handle();
    static void GET_Data();

};
#endif //RM_FRAME_C_CATCHCONTROL_H
