//
// Created by LEGION on 2021/10/4.
//
#include "ControlTask.h"


void CtrlHandle() {
    if (RemoteControl::rcInfo.sRight == DOWN_POS) {//右侧三档，急停模式
        ChassisStop();
        ArmStop();
    } else {//其他正常模式
        switch (RemoteControl::rcInfo.sLeft) {
            case UP_POS://左侧一档{
                if (RemoteControl::rcInfo.sRight == UP_POS) {
                    ChassisSetVelocity(RemoteControl::rcInfo.right_col * 2,
                                       RemoteControl::rcInfo.right_rol * 2, RemoteControl::rcInfo.left_rol);
                    ArmSetAngle(RemoteControl::rcInfo.left_col, RemoteControl::rcInfo.left_col * 90);
                    Headmemory();
                }else if (RemoteControl::rcInfo.sRight == MID_POS){
                    StateMachine::stateHandle();
                   // AutoSetVelocity();
                }
                break;
            case MID_POS://左侧二档
                if (RemoteControl::rcInfo.sRight == UP_POS) {
                    HeadlessSetVelocity(RemoteControl::rcInfo.right_col * 2,
                                        RemoteControl::rcInfo.right_rol * 2, RemoteControl::rcInfo.left_rol);
                } else if (RemoteControl::rcInfo.sRight == MID_POS) {
                    HeadkeepSetVelocity(RemoteControl::rcInfo.right_col * 2,
                                        RemoteControl::rcInfo.right_rol * 2, RemoteControl::rcInfo.left_rol);
                }
                break;
            case DOWN_POS:
                if (RemoteControl::rcInfo.sRight == UP_POS) {
                AutoSetVelocity();
                }
                break;
            default:
                break;
        }

    }

}