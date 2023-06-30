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
/*枚举类型定义------------------------------------------------------------*/
typedef enum {
    STOP = 1,
    MOVE,
    ARM,
    TRAY,
    CLAW,
}TASK_FLAG_t;
/*结构体定义--------------------------------------------------------------*/
//可根据需要创建需要的结构体
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
/*类定义------------------------------------------------------------------*/
class ManiControl : public Device {
public:
    static MC_ctrl_t mc_ctrl;
    static uint8_t rx_buff[2][BUFF_SIZE];
    static TASK_FLAG_t TaskFlag;

    static void Init();//串口通信初始化函数，对应UART6
    static void GetData(uint8_t bufIndex);//处理串口数据
    static void IT_Handle();//串口中断处理函数，使用双缓冲接收

};
void CompleteTask();//完成任务反馈函数,一般为向上位机发送数据0x01

//以下为直接调用UART6中断函数，在实际使用中发现会跳过DMA中断处理函数，导致DMA数据发送中断，目前不建议使用，若无DMA发送需求可使用，降低单片机功耗
/*
#ifdef __cplusplus
extern "C" {
#endif
extern void USART6_IRQHandler(void);
#ifdef __cplusplus
}
#endif
*/

#endif //RM_FRAME_C_MANICONTROL_H
