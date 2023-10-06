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
    if (RemoteControl::rcInfo.sRight == DOWN_POS) {//右侧三档，急停模式
        ChassisStop();
    } 
    // else if (RemoteControl::rcInfo.sRight == MID_POS){//右侧二档，脉冲模式
    //     ChassisSetVelocity(0.4,0,0);
    // }
    else {//其他正常模式
        switch (RemoteControl::rcInfo.sLeft) {
            case UP_POS://左侧一档{
                if (RemoteControl::rcInfo.sRight == UP_POS) {
                    ChassisSetVelocity(RemoteControl::rcInfo.right_col * 2,
                                       RemoteControl::rcInfo.right_rol * 2, RemoteControl::rcInfo.left_rol * 2 * PI);
                    Headmemory();
                }/*else if (RemoteControl::rcInfo.sRight == MID_POS){
                    AutoSetVelocity();
                }*/
                break;
            case MID_POS://左侧二档
                if (RemoteControl::rcInfo.sRight == UP_POS) {
                    // HeadlessSetVelocity(RemoteControl::rcInfo.right_col * 2,
                    //                     RemoteControl::rcInfo.right_rol * 2, RemoteControl::rcInfo.left_rol);
                    ChassisDistanceSet(0, 0, PI / 2);
                    
                } else if (RemoteControl::rcInfo.sRight == MID_POS) {
                    ChassisDistanceSet(0, 1, 0);
                }
                //ChassisSetVelocity(0,0.4,0);
               
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
/*
void CtrlHandle() {
    if (RemoteControl::rcInfo.sRight == DOWN_POS) {//右侧三档，急停模式
        ChassisStop();
        ArmStop();
    } else {//其他正常模式
        switch (RemoteControl::rcInfo.sLeft) {
            case UP_POS://左侧一档{
                if (RemoteControl::rcInfo.sRight == UP_POS) {
                    ChassisSetVelocity(RemoteControl::rcInfo.right_col * 8,
                                       RemoteControl::rcInfo.right_rol * 8, RemoteControl::rcInfo.left_rol*2);
                    //ArmSet(RemoteControl::rcInfo.left_col, RemoteControl::rcInfo.left_col * 90,RemoteControl::rcInfo.left_col);
                    AutoClawSet(0);
                    Headmemory();
                }else if (RemoteControl::rcInfo.sRight == MID_POS){
                    ChassisSetVelocity(RemoteControl::rcInfo.right_col * 8,
                                       RemoteControl::rcInfo.right_rol * 8, RemoteControl::rcInfo.left_rol*2);
                    AutoClawSet(1);
                }
                break;
            case MID_POS://左侧二档
                if (RemoteControl::rcInfo.sRight == UP_POS) {
                    ArmSet(RemoteControl::rcInfo.right_rol * -1.57f,
                           RemoteControl::rcInfo.left_rol * -130,0);
                    AutoClawSet(0);
                } else if (RemoteControl::rcInfo.sRight == MID_POS) {
                    ArmSet(RemoteControl::rcInfo.right_rol* -1.57f,
                           RemoteControl::rcInfo.left_rol * -130,0);
                    AutoClawSet(1);
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

}*/
