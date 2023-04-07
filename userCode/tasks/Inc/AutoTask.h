//
// Created by 25396 on 2023/4/6.
//

#ifndef RM_FRAME_C_AUTOTASK_H
#define RM_FRAME_C_AUTOTASK_H
#include "Device.h"
#include "StateMachine.h"
#include "ManiControl.h"


static void AutoTask();
//状态函数声明
void ChassisStopTask();
void MoveTask();
void ArmTask();
void TrayTask();
void ClawTask();

/*外部函数声明-------------------------------------------------------------*/
extern void AutoChassisStop();//Realized in ChassisTask
extern void AutoChassisSet(uint16_t x,uint16_t y,uint16_t o);//Realized in ChassisTask
extern void AutoArmSet(uint16_t angle1,uint16_t angle2,uint8_t pos);//Realized in ArmTask
extern void AutoTraySet(uint8_t trayflag);//Realized in ArmTask
extern void AutoClawSet(uint8_t clawflag);//Realized in ServoTask

#endif //RM_FRAME_C_AUTOTASK_H
