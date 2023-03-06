//
// Created by mac on 2022/12/14.
//

#ifndef RM_FRAME_C_CATCHCONTROL_H
#define RM_FRAME_C_CATCHCONTROL_H


#include "Device.h"

#define BUFF_SIZE 28
#define CONTROL_LENGTH 14u
typedef enum {
    STOP = 0,
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
} CC_ctrl_t;

class CatchControl : public Device {
public:
    static CC_ctrl_t cc_ctrl;
    static uint16_t data_length;
    static uint8_t rx_buff[2][BUFF_SIZE];
    static TASK_FLAG_t TaskFlag;

    static void Init();
    static void GetData(uint8_t bufIndex);
    static void AutoTask();
    static void IT_Handle();

  //  static void GET_Data(const volatile uint8_t *buf);

};

/*外部函数声明-------------------------------------------------------------*/
extern void AutoChassisStop();//Realized in ChassisTask
extern void AutoChassisSet(uint16_t x,uint16_t y);//Realized in ChassisTask
extern void AutoArmSet(uint16_t angle1,uint16_t angle2,uint8_t pos);//Realized in ArmTask
void AutoTraySet(uint8_t trayflag);//Realized in ServoTask
void AutoClawSet(uint8_t clawflag);//Realized in ServoTask

#ifdef __cplusplus
extern "C" {
#endif
extern void USART6_IRQHandler(void);
#ifdef __cplusplus
}
#endif

#endif //RM_FRAME_C_CATCHCONTROL_H
