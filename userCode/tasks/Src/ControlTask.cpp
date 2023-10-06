#include "ControlTask.h"

void CtrlHandle() {
    if (RemoteControl::rcInfo.sRight == DOWN_POS) {//右侧三档，急停模式
        Chassis::Instance().Stop();
    } 
    // else if (RemoteControl::rcInfo.sRight == MID_POS){//右侧二档，脉冲模式
    //     ChassisSetVelocity(0.4,0,0);
    // }
    else {//其他正常模式
        switch (RemoteControl::rcInfo.sLeft) {
            case UP_POS://左侧一档{
                if (RemoteControl::rcInfo.sRight == UP_POS) {
                    Chassis::Instance().SetTargetVelocity({RemoteControl::rcInfo.right_rol * 0.5f,RemoteControl::rcInfo.right_col * 0.5f, -RemoteControl::rcInfo.left_rol * 1.0f});
//                    Headmemory();
                }/*else if (RemoteControl::rcInfo.sRight == MID_POS){
                    AutoSetVelocity();
                }*/
                break;
            case MID_POS://左侧二档
                if (RemoteControl::rcInfo.sRight == UP_POS) {
                    // HeadlessSetVelocity(RemoteControl::rcInfo.right_col * 2,
                    //                     RemoteControl::rcInfo.right_rol * 2, RemoteControl::rcInfo.left_rol);
//                    ChassisDistanceSet(0, 0, PI/2);
                    //Chassis::Instance().SetTargetDistance({0,0.5,0});
                    
                } else if (RemoteControl::rcInfo.sRight == MID_POS) {
//                    ChassisDistanceSet(0, 1, 0);
                }
                //ChassisSetVelocity(0,0.4,0);
               
                break;
            case DOWN_POS:
                if (RemoteControl::rcInfo.sRight == UP_POS) {
                    //Chassis::Instance().CalcVelocity();
                }
                
                break;
            default:
                break;
        }

    }

}
