//
// Created by mac on 2022/12/14.
//

#ifndef RM_FRAME_C_MANICONTROL_H
#define RM_FRAME_C_MANICONTROL_H
#include "Device.h"
#include "StateMachine.h"
#include "AutoTask.h"
#define BUFF_SIZE 28u
#define CONTROL_LENGTH 0x10
#define MANI_LENGTH 0x0E
typedef enum {
    STOP = 1,
    MOVE,
    ARM,
    TRAY,
    CLAW,
}TASK_FLAG_t;
typedef struct {
    uint16_t speed;
    uint16_t angle;
} ARM_col_t;
typedef struct {
    ARM_col_t ARM1;
    ARM_col_t ARM2;
    uint8_t ARM_Z_Flag;
    uint16_t x;
    uint16_t y;
    uint8_t TrayFlag;
    uint8_t ChassisStopFlag;
    uint8_t ArmServoFlag;
} MC_ctrl_t;

class ManiControl : public Device {
public:
    static MC_ctrl_t mc_ctrl;
    static uint8_t rx_buff[2][BUFF_SIZE];
    static TASK_FLAG_t TaskFlag;

    static void Init();
    static void GetData(uint8_t bufIndex);
    static void IT_Handle();

};
void CompleteTask();

#ifdef __cplusplus
extern "C" {
#endif
extern void USART6_IRQHandler(void);
#ifdef __cplusplus
}
#endif

#endif //RM_FRAME_C_MANICONTROL_H
