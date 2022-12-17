//
// Created by LEGION on 2021/10/4.
//
#include "ControlTask.h"


void CtrlHandle(){
    if (RemoteControl::rcInfo.sRight == DOWN_POS){//右侧三档，急停模式
        ChassisStop();
    }else {//其他正常模式
        switch (RemoteControl::rcInfo.sLeft) {
            case UP_POS://左侧一档
                ChassisSetVelocity(RemoteControl::rcInfo.right_col*2,
                                   RemoteControl::rcInfo.right_rol*2,RemoteControl::rcInfo.left_rol*60);
                Headmemory();
                break;
            case MID_POS://左侧二档
                if (RemoteControl::rcInfo.sRight == UP_POS){
                    HeadlessSetVelocity(RemoteControl::rcInfo.right_col*2,
                                        RemoteControl::rcInfo.right_rol*2,RemoteControl::rcInfo.left_rol*60);
                }else if(RemoteControl::rcInfo.sRight == MID_POS) {
                    HeadkeepSetVelocity(RemoteControl::rcInfo.right_col*2,
                                        RemoteControl::rcInfo.right_rol*2,RemoteControl::rcInfo.left_rol*60);
                }
				break;
			case DOWN_POS:
                if (RemoteControl::rcInfo.sRight == UP_POS){
                    AutoSetVelocity();
                }
                break;
			default:
			    break;
        }

    }

}