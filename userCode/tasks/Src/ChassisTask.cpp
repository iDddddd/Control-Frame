//
// Created by LEGION on 2021/10/4.
//
#include "ChassisTask.h"


constexpr float L = 0.2f; //车身长
constexpr float M = 0.2f; //车身宽

PID_Regulator_t pidRegulator1 = {//此为储存pid参数的结构体，四个底盘电机共用
        .kp = -2.0f,
        .ki = -0.0002f,
        .kd = 0,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000 //3508电机输出电流上限，可以调小，勿调大
};
PID_Regulator_t pidRegulator2 = {//此为储存pid参数的结构体，四个底盘电机共用
        .kp = 1.5f,
        .ki = 0.0002f,
        .kd = 0,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000 //3508电机输出电流上限，可以调小，勿调大
};
MOTOR_INIT_t chassisMotorInit1 = {//四个底盘电机共用的初始化结构体
        .speedPIDp = &pidRegulator1,
        .anglePIDp = nullptr,
        ._motorID = MOTOR_ID_1,
        .reductionRatio = 1.0f,
        .ctrlType = SPEED_Single,
        .commuType = CAN,
};
MOTOR_INIT_t chassisMotorInit2 = {//四个底盘电机共用的初始化结构体
        .speedPIDp = &pidRegulator2,
        .anglePIDp = nullptr,
        ._motorID = MOTOR_ID_1,
        .reductionRatio = 1.0f,
        .ctrlType = SPEED_Single,
        .commuType = CAN,
};

MOTOR_INIT_t swerveMotorInit = {//四个底盘电机共用的初始化结构体
        .speedPIDp = nullptr,
        .anglePIDp = nullptr,
        ._motorID = MOTOR_ID_1,
        .reductionRatio = 1.0f,
        .ctrlType = DIRECT,
        .commuType = RS485,
};

Motor CMFL(MOTOR_ID_1,&chassisMotorInit1);//定义左前轮电机
Motor CMFR(MOTOR_ID_2,&chassisMotorInit1);//定义右前轮电机
Motor CMBL(MOTOR_ID_4,&chassisMotorInit2);//定义左后轮电机
Motor CMBR(MOTOR_ID_3,&chassisMotorInit2);//定义右后轮电机

Motor RFL(MOTOR_ID_1, &swerveMotorInit);
Motor RFR(MOTOR_ID_2, &swerveMotorInit);
Motor RBL(MOTOR_ID_4, &swerveMotorInit);
Motor RBR(MOTOR_ID_3, &swerveMotorInit);

bool ChassisStopFlag = true;
float FBVelocity,LRVelocity,RTVelocity;

void ChassisStart(){

}
/**
 * @brief 底盘任务的处理函数，定时执行
 * @callergraph void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) in Device.cpp
 */
void ChassisHandle() {
    if(ChassisStopFlag == false) {
        WheelsSpeedCalc(FBVelocity, LRVelocity, RTVelocity);
    }
    CMFL.Handle();
    CMFR.Handle();
    CMBL.Handle();
    CMBR.Handle();

    RFL.Handle();
    RBL.Handle();
    RFR.Handle();
    RBR.Handle();
}
/**
 * @brief 用于控制任务控制底盘速度
 * @param _fbV 底盘前后方向速度
 * @param _lrV 底盘左右方向速度
 * @param _rtV 底盘旋转速度
 */
void ChassisSetVelocity(float _fbV,float _lrV,float _rtV){
    ChassisStopFlag = false;
    FBVelocity = _fbV;
    LRVelocity = _lrV;
    RTVelocity = _rtV;
}

/**
 * @brief 执行急停模式的底盘任务处理
 */
void ChassisStop(){
    ChassisStopFlag = true;
    CMFL.Stop();
    CMFR.Stop();
    CMBL.Stop();
    CMBR.Stop();
}
int sign(float x){
    if (x<0) return -1;
    if (x>0) return 1;
    return 0;
}
float SetAngle(float Angle){
    if (Angle > 270.0) {
		Angle -= 180.0;
		}
    if (Angle < 90.0){
		Angle += 180.0;
		}
    return Angle;
}

/**
 * @brief 速度与角度计算任务
 * @param fbVelocity
 * @param lrVelocity
 * @param rtVelocity
 */
void WheelsSpeedCalc(float fbVelocity, float lrVelocity, float rtVelocity) {
    float CMFLSpeed, CMFRSpeed, CMBLSpeed, CMBRSpeed;
    float RFLAngle, RFRAngle, RBLAngle, RBRAngle;
    rtVelocity = RPM2RADpS(rtVelocity);

    //计算四个轮子线速度，单位：m/s
    /**
     * @brief 此处四句代码需要结合底盘的三个速度，计算处四个轮子的位置对应的线速度。
     * @param fbVelocity,lrVelocity,rtVelocity
     * @return CMFLSpeed CMFRSpeed CMBLSpeed CMBRSpeed
     */
//    CMFLSpeed = fbVelocity - rtVelocity;
//    CMFRSpeed = -fbVelocity - rtVelocity;
//    CMBLSpeed = -fbVelocity + rtVelocity;
//    CMBRSpeed = fbVelocity + rtVelocity;

    RFLAngle =  -atan2( (lrVelocity - rtVelocity * L / 2), (fbVelocity - rtVelocity * M / 2) ) * 180 / 3.1415926f + 180;
    RFRAngle =  -atan2( (lrVelocity - rtVelocity * L / 2), (fbVelocity + rtVelocity * M / 2) ) * 180 / 3.1415926f + 180;
    RBLAngle =  -atan2( (lrVelocity + rtVelocity * L / 2), (fbVelocity - rtVelocity * M / 2) ) * 180 / 3.1415926f + 180;
    RBRAngle =  -atan2( (lrVelocity + rtVelocity * L / 2), (fbVelocity + rtVelocity * M / 2) ) * 180 / 3.1415926f + 180;


    RFL.SetTargetAngle( SetAngle(RFLAngle));
    RFR.SetTargetAngle( SetAngle(RFRAngle));
    RBL.SetTargetAngle( SetAngle(RBLAngle));
    RBR.SetTargetAngle( SetAngle(RBRAngle));

    CMFLSpeed = -sign(fbVelocity - rtVelocity * M /2) * sqrt((lrVelocity - rtVelocity * L / 2) * (lrVelocity - rtVelocity * L / 2) +
                     (fbVelocity - rtVelocity * M / 2) * (fbVelocity - rtVelocity * M / 2));
    CMFRSpeed = sign(fbVelocity + rtVelocity * M /2) * sqrt((lrVelocity - rtVelocity * L / 2) * (lrVelocity - rtVelocity * L / 2) +
                     (fbVelocity + rtVelocity * M / 2) * (fbVelocity + rtVelocity * M / 2));
    CMBLSpeed = -sign(fbVelocity - rtVelocity * M /2) * sqrt((lrVelocity + rtVelocity * L / 2) * (lrVelocity + rtVelocity * L / 2) +
                     (fbVelocity - rtVelocity * M / 2) * (fbVelocity - rtVelocity * M / 2));
    CMBRSpeed = sign(fbVelocity + rtVelocity * M /2) * sqrt((lrVelocity + rtVelocity * L / 2) * (lrVelocity + rtVelocity * L / 2) +
                     (fbVelocity + rtVelocity * M / 2) * (fbVelocity + rtVelocity * M / 2));
    //计算四个轮子角速度，单位：rad/s
    CMFLSpeed = CMFLSpeed /(WHEEL_DIAMETER/2.0f);
    CMFRSpeed = CMFRSpeed /(WHEEL_DIAMETER/2.0f);
    CMBLSpeed = CMBLSpeed /(WHEEL_DIAMETER/2.0f);
    CMBRSpeed = CMBRSpeed /(WHEEL_DIAMETER/2.0f);
    //控制底盘电机转速
    CMFL.SetTargetSpeed(CMFLSpeed * 180 / 3.1415926f);
    CMFR.SetTargetSpeed(CMFRSpeed * 180 / 3.1415926f);
    CMBL.SetTargetSpeed(CMBLSpeed * 180 / 3.1415926f);
    CMBR.SetTargetSpeed(CMBRSpeed * 180 / 3.1415926f);
}
