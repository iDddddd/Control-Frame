//
// Created by LEGION on 2021/10/4.
//

#ifndef RM_FRAME_C_DEVICE_H
#define RM_FRAME_C_DEVICE_H

#include "main.h"

#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"


#include <cstdarg>
#include <cstring>
#include <cmath>
#include <cstdio>
#include "usbd_cdc_if.h"

#define INRANGE(NUM, MIN, MAX) \
{\
    if(NUM<MIN){\
        NUM=MIN;\
    }else if(NUM>MAX){\
        NUM=MAX;\
    }\
}
#define PI 3.1415926f

/*枚举类型定义------------------------------------------------------------*/

extern void ChassisHandle();

extern void CtrlHandle();

extern void ARMHandle();

extern void ServoHandle();

/*
 * 设备类型枚举
 */
typedef enum {
    MOTOR,
    SERVO,
} DEVICE_TYPE_E;

/*结构体定义--------------------------------------------------------------*/
typedef struct {
    uint32_t robot_ID;
    uint32_t yaw_zero;
    uint32_t pitch_zero;
    uint32_t pat[125];
} flash_data_t;
/*联合体定义--------------------------------------------------------------*/
typedef union {
    uint8_t u8[4];
    int32_t i32;
    float f;
} f_u8_t;
/*类型定义----------------------------------------------------------------*/

class Device {
protected:
    DEVICE_TYPE_E deviceType;
    uint32_t deviceID;

    virtual void Handle() = 0;

    virtual void ErrorHandle() = 0;

};

/*结构体成员取值定义组------------------------------------------------------*/
/*外部变量声明-------------------------------------------------------------*/
/*外部函数声明-------------------------------------------------------------*/



#endif //RM_FRAME_C_DEVICE_H
