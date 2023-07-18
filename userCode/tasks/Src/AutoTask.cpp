//
// Created by 25396 on 2023/4/6.
//

#include "AutoTask.h"



void ChassisStopTask(){
    AutoChassisStop();
    StateMachine::remove_function_from_state(ChassisStopTask);
    CompleteTask();
}
void MoveTask(){
    AutoChassisSet(ManiControl::mc_ctrl.x_Dis.f, ManiControl::mc_ctrl.y_Dis.f, 0);
    StateMachine::remove_function_from_state(MoveTask);
}
void ArmTask(){
    AutoArmSet(ManiControl::mc_ctrl.ARMZ_Pos.f, ManiControl::mc_ctrl.ARM1_Pos.f, ManiControl::mc_ctrl.ARM2_Pos.f);
    StateMachine::remove_function_from_state(ArmTask);
}
void TrayTask(){
    AutoTraySet(ManiControl::mc_ctrl.TrayFlag);
    StateMachine::remove_function_from_state(TrayTask);
}
void ClawTask(){
    AutoClawSet(ManiControl::mc_ctrl.ClawFlag);
    StateMachine::remove_function_from_state(ClawTask);
}
