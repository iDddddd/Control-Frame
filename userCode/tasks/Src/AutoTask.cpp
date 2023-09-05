//
// Created by 25396 on 2023/4/6.
//

#include "AutoTask.h"



void ChassisStopTask(){
    AutoChassisStop();
    StateMachine::remove_function_from_state(ChassisStopTask);
    CompleteTask();
}
void Move_DisTask(){
    ChassisDistanceSet(ManiControl::mc_ctrl.chassisDis_col.x_Dis.f, ManiControl::mc_ctrl.chassisDis_col.y_Dis.f, ManiControl::mc_ctrl.chassisDis_col.Theta.f);
    StateMachine::remove_function_from_state(Move_DisTask);
}
void Move_VelTask(){
    ChassisVelocitySet(ManiControl::mc_ctrl.chassisVel_col.x_Vel.f, ManiControl::mc_ctrl.chassisVel_col.y_Vel.f, ManiControl::mc_ctrl.chassisVel_col.w_Vel.f);
    StateMachine::remove_function_from_state(Move_VelTask);
}
void ArmTask(){
    ArmSet(ManiControl::mc_ctrl.arm_col.Joint1Pos.f,ManiControl::mc_ctrl.arm_col.Joint2Pos.f,ManiControl::mc_ctrl.arm_col.Joint3Pos.f,ManiControl::mc_ctrl.arm_col.Joint4Pos.f,ManiControl::mc_ctrl.arm_col.Joint5Pos.f);
    StateMachine::remove_function_from_state(ArmTask);
}
void TrayTask(){
   // AutoTraySet(ManiControl::mc_ctrl.TrayFlag);
    StateMachine::remove_function_from_state(TrayTask);
}
void ClawTask(){
    AutoClawSet(ManiControl::mc_ctrl.ClawFlag);
    StateMachine::remove_function_from_state(ClawTask);
}
