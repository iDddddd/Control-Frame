//
// Created by LEGION on 2021/10/4.
//
#include "ControlTask.h"

static int flag = 0;
void autoImpulse(){
    if (flag>1000){
        chassis.ChassisSetVelocity(1,0,0);
        flag++;
        if(flag>1000*2) flag=0;
    }
    else{
        chassis.ChassisSetVelocity(0,0,0);
        flag++;
    }    
}

void CtrlHandle() {
    if (RemoteControl::rcInfo.sRight == DOWN_POS) {//右侧三档，急停模式
        chassis.ChassisStop();
        ArmStop();
    } 
    // else if (RemoteControl::rcInfo.sRight == MID_POS){//右侧二档，脉冲模式
    //     ChassisSetVelocity(0.4,0,0);
    // }
    else {//其他正常模式
        switch (RemoteControl::rcInfo.sLeft) {
            case UP_POS://左侧一档{
                if (RemoteControl::rcInfo.sRight == UP_POS) {
                    chassis.ChassisSetVelocity(RemoteControl::rcInfo.right_col * 2,
                                       RemoteControl::rcInfo.right_rol * 2, RemoteControl::rcInfo.left_rol);
                    chassis.Headmemory();
                }/*else if (RemoteControl::rcInfo.sRight == MID_POS){
                    AutoSetVelocity();
                }*/
                break;
            case MID_POS://左侧二档
                if (RemoteControl::rcInfo.sRight == UP_POS) {
                    // HeadlessSetVelocity(RemoteControl::rcInfo.right_col * 2,
                    //                     RemoteControl::rcInfo.right_rol * 2, RemoteControl::rcInfo.left_rol);
                    chassis.ChassisDistanceSet(0, 0, PI/2);
                    
                } else if (RemoteControl::rcInfo.sRight == MID_POS) {
                    chassis.ChassisDistanceSet(0, 0.05, 0);
                }
                //ChassisSetVelocity(0,0.4,0);
               
                break;
            case DOWN_POS:
                if (RemoteControl::rcInfo.sRight == UP_POS) {
                    chassis.AutoSetVelocity();
                }
                
                break;
            default:
                break;
        }

    }

}
