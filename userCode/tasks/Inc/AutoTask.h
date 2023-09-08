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
void Move_DisTask();
void ArmTask();
void TrayTask();
void ClawTask();
void Move_VelTask();

/*外部函数声明-------------------------------------------------------------*/
extern void AutoChassisStop();//Realized in ChassisTask
extern void ChassisDistanceSet(float x, float y, float o);//Realized in ChassisTask
extern void ChassisVelocitySet(float x_vel, float y_vel, float w_vel);//Realized in ChassisTask
extern void ArmSet(float Joint1Pos, float Joint2Pos, float Joint3Pos, float Joint4Pos, float Joint5Pos);
//extern void AutoTraySet(uint8_t trayflag);//Realized in ArmTask
extern void ClawSet(uint8_t clawflag);//Realized in ArmTask

#endif //RM_FRAME_C_AUTOTASK_H
