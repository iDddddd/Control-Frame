//
// Created by LEGION on 2021/10/4.
//
#include "ControlTask.h"

static int flag = 0;
void autoImpulse(){
    if (flag>1000){
        ChassisSetVelocity(1,0,0);
        flag++;
        if(flag>1000*2) flag=0;
    }
    else{
        ChassisSetVelocity(0,0,0);
        flag++;
    }    
}

void CtrlHandle() {

   // StateMachine::stateHandle();

 /*   if (RemoteControl::rcInfo.sRight == DOWN_POS) {//右侧三档，急停模式
        ChassisStop();
        ArmTask::ArmStop();
    } 
*//*    else if (RemoteControl::rcInfo.sRight == MID_POS){//右侧二档，脉冲模式
        autoImpulse();
    }*//*
    else {//其他正常模式
        switch (RemoteControl::rcInfo.sLeft) {
            case UP_POS://左侧一档{
                if (RemoteControl::rcInfo.sRight == UP_POS) {
                    ChassisSetVelocity(RemoteControl::rcInfo.right_col * 2,
                                       RemoteControl::rcInfo.right_rol * 2, RemoteControl::rcInfo.left_rol);
                    Headmemory();
                }else if (RemoteControl::rcInfo.sRight == MID_POS){
                    StateMachine::stateHandle();
                    AutoSetVelocity();
                }
                break;
            case MID_POS://左侧二档
                if (RemoteControl::rcInfo.sRight == UP_POS) {
                    HeadlessSetVelocity(RemoteControl::rcInfo.right_col * 2,
                                        RemoteControl::rcInfo.right_rol * 2, RemoteControl::rcInfo.left_rol);
                   // ChassisDistanceSet(0, 0, PI / 2);
                } else if (RemoteControl::rcInfo.sRight == MID_POS) {
                    HeadkeepSetVelocity(RemoteControl::rcInfo.right_col * 2,
                                        RemoteControl::rcInfo.right_rol * 2, RemoteControl::rcInfo.left_rol);
                }
                break;
            case DOWN_POS:
                if (RemoteControl::rcInfo.sRight == UP_POS) {

                }
                break;
            default:
                break;
        }

    }*/

}
