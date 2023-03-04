//
// Created by LEGION on 2021/10/5.
//

#ifndef RM_FRAME_C_SERVO_H
#define RM_FRAME_C_SERVO_H

#include "Device.h"



/*结构体定义--------------------------------------------------------------*/
/*
 * 舵机初始化结构体
 */
typedef struct {
    uint32_t servoID;
    float firstAngle, angleLimit_Min, angleLimit_Max;//only valid for POSITION_180 servo
} SERVO_INIT_T;

typedef struct {
    TIM_HandleTypeDef *handleTypeDef;
    uint32_t timChannel;
} SERVO_HANDLE_T;

/*类型定义----------------------------------------------------------------*/
class Servo : public Device {
    static uint32_t servo_IDs;

    static void Init();

    uint8_t stopFlag{1};

    float angleLimit_Min, angleLimit_Max;

    float duty{0};
    float targetAngle{0};

    void ErrorHandle() override;
public:
    explicit Servo(SERVO_INIT_T *servoInit);

    void stop();

    void Handle() override;

    void SetTargetAngle(float _targetAngle);
};
/*结构体成员取值定义组------------------------------------------------------*/

/*
 * SEVRO_ID
 */
#define SERVO_ID_1 0x00
#define SERVO_ID_2 0x01


/*外部变量声明-------------------------------------------------------------*/

/*外部函数声明-------------------------------------------------------------*/
#endif //RM_FRAME_C_SERVO_H
