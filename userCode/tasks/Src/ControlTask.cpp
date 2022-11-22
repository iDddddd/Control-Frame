//
// Created by LEGION on 2021/10/4.
//
#include "ControlTask.h"


void CtrlHandle(){
    if (RemoteControl::rcInfo.sRight == DOWN_POS){//右侧三档，急停模式
        ChassisStop();
        UserStop();
    }else {//其他正常模式
        switch (RemoteControl::rcInfo.sLeft) {
            case UP_POS://左侧一档
                ChassisSetVelocity(RemoteControl::rcInfo.right_col*2,
                                   RemoteControl::rcInfo.right_rol*2,RemoteControl::rcInfo.left_rol*60);
                break;
            case MID_POS://左侧二档
                uint8_t clawState;
                if (RemoteControl::rcInfo.sRight == UP_POS){
                    clawState = 0;
                }else if(RemoteControl::rcInfo.sRight == MID_POS) {
                    clawState = 1;
                }
				break;
			case DOWN_POS:default:
				break;
        }

    }

}