//
// Created by David9686 on 2022/12/16.
//

#include "ServoTask.h"


SERVO_INIT_T ArmServo = {
        .servoType = POSITION_180,
        .servoID = SERVO_ID_1,
        .firstAngle = 900,
        .angleLimit_Min = 0,
        .angleLimit_Max = 2200,
};

SERVO_INIT_T TrayServo = {
        .servoType = POSITION_180,
        .servoID = SERVO_ID_2,
        .firstAngle = 930,
        .angleLimit_Min = 800,
        .angleLimit_Max = 2200,
};

Servo armServo(&ArmServo);
Servo trayServo(&TrayServo);


/*void ServoSetAngle(){
//    if(CatchControl::cc_ctrl.ArmServoFlag == 1) {
//        armServo.SetTargetAngle(1300);
//    } else if (CatchControl::cc_ctrl.ArmServoFlag == 0)
//    {
//        armServo.SetTargetAngle(1700);
//    }
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,
                          htim1.Instance->ARR*1300);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,
                          htim1.Instance->ARR*930);
}*/
void ServoHandle(){
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,
                          1300);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,
                          930);
}


