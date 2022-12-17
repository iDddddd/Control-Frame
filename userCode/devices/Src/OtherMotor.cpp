//
// Created by mac on 2022/12/14.
//

#include "OtherMotor.h"

uint8_t ARMMotor::arm1message[8] = {0};
uint8_t ARMMotor::arm2message[8] = {0};
uint8_t ARMMotor::arm1_Initmessage[2] = {0x30, 0x6B};
uint8_t ARMMotor::arm2_Initmessage[8] = {0xFF, 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, 0xFC};
uint8_t TRAYMotor::traymessage[3][8] = {{0xA4, 0x00, 0x2C, 0x01, 0x90, 0x65, 0x00, 0x00},
                                        {0xA4, 0x00, 0x2C, 0x01, 0xB0, 0x36, 0x00, 0x00},
                                        {0xA4, 0x00, 0x2C, 0x01, 0xD0, 0x07, 0x00, 0x00}};
uint8_t TRAYMotor::trayflag;// 1 or 2 or 3
float ARMMotor::feedback_moment[3];
static uint8_t Arm1SendFlag = 0;



void ARMMotor::Init(){
    //ARM1_Init();
    ARM2_Init();
}
/**
 * @brief 步进电机初始化，读取编码值
 */
void ARMMotor::ARM1_Init(){
    CAN_TxHeaderTypeDef txHeaderTypeDef;

    txHeaderTypeDef.StdId = 0x01;
    txHeaderTypeDef.DLC = 0x08;
    txHeaderTypeDef.IDE = CAN_ID_STD;
    txHeaderTypeDef.RTR = CAN_RTR_DATA;
    txHeaderTypeDef.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(&hcan2, &txHeaderTypeDef,arm1_Initmessage,0);
}

/**
 * @brief 机械臂can步进电机消息获取
 */
void ARMMotor::ARMCAN1MessageGenerate(){
    uint8_t Arm1MoveFlag = 2;// CatchControl::cc.ctrl.ARM1.Flag
    if(Arm1MoveFlag == 1) {
        arm1message[0] = 0xFD;
        arm1message[1] = 0x12;//1代表正转，0代表反转
        arm1message[2] = 0xFF;//4FF代表速度
        arm1message[3] = 0x00;//加速度，0为不启用,若需较快，改为0x96
        arm1message[4] = CatchControl::cc_ctrl.ARM1.angle;//0x80
        arm1message[5] = CatchControl::cc_ctrl.ARM1.angle >> 8u;//0x0C
        arm1message[6] = CatchControl::cc_ctrl.ARM1.angle >> 16u;//4~6为脉冲数,3200为一圈，对应z轴8mm
        arm1message[7] = 0x6B;
    }
    else if(Arm1MoveFlag == 0){
        arm1message[0] = 0xFD;
        arm1message[1] = 0x02;//1代表正转，0代表反转
        arm1message[2] = 0xFF;//4FF代表速度
        arm1message[3] = 0x00;//加速度，0为不启用,若需较快，改为0x96
        arm1message[4] = CatchControl::cc_ctrl.ARM1.angle;//0x80
        arm1message[5] = CatchControl::cc_ctrl.ARM1.angle >> 8u;//0x0C
        arm1message[6] = CatchControl::cc_ctrl.ARM1.angle >> 16u;//4~6为脉冲数,3200为一圈，对应z轴8mm
        arm1message[7] = 0x6B;
    }
    else if (Arm1MoveFlag == 0){
        arm1message[0] = 0xFD;
        arm1message[1] = 0x12;//1代表正转，0代表反转
        arm1message[2] = 0xFF;//4FF代表速度
        arm1message[3] = 0x00;//加速度，0为不启用,若需较快，改为0x96
        arm1message[4] = 00;//0x80
        arm1message[5] = 00;//0x0C
        arm1message[6] = 00;//4~6为脉冲数,3200为一圈，对应z轴8mm
        arm1message[7] = 0x6B;
    }

}
/**
 * @brief 机械臂can步进电机发送任务
 */
void ARMMotor::ARMCAN1PackageSend(){
    CAN_TxHeaderTypeDef txHeaderTypeDef;

    txHeaderTypeDef.StdId = 0x01;
    txHeaderTypeDef.DLC = 0x08;
    txHeaderTypeDef.IDE = CAN_ID_STD;
    txHeaderTypeDef.RTR = CAN_RTR_DATA;
    txHeaderTypeDef.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(&hcan2, &txHeaderTypeDef,arm1message,0);
}
/**
 * @brief 机械臂can4310初始化任务，将电机使能
 */
void ARMMotor::ARM2_Init(){
    CAN_TxHeaderTypeDef txHeaderTypeDef;

    txHeaderTypeDef.StdId = 0x101;
    txHeaderTypeDef.DLC = 0x08;
    txHeaderTypeDef.IDE = CAN_ID_STD;
    txHeaderTypeDef.RTR = CAN_RTR_DATA;
    txHeaderTypeDef.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(&hcan2, &txHeaderTypeDef,arm2_Initmessage,0);
}
/**
 * @brief 机械臂can4310电机消息获取
 */
void ARMMotor::ARMCAN2MessageGenerate(){
//    uint32_t tmp = motor_intensity[1][0];
    arm2message[0] = 0x00;//位置低字节
    arm2message[1] = 0x00;
//    arm2message[2] = CatchControl::cc_ctrl.ARM2.angle;
//    arm2message[3] = CatchControl::cc_ctrl.ARM2.angle >> 8u;//位置高字节
    arm2message[2] = 0x00;
    arm2message[3] = 0x00;//位置高字节
    arm2message[4] = 0x00;//速度低字节
    arm2message[5] = 0x00;
//    arm2message[6] = CatchControl::cc_ctrl.ARM2.angle ;
//    arm2message[7] = CatchControl::cc_ctrl.ARM2.angle >> 8u;//速度高字节
    arm2message[6] = 0x00 ;
    arm2message[7] = 0x3F;//速度高字节

}
/**
 * @brief 机械臂can4310电机发送任务
 */
void ARMMotor::ARMCAN2PackageSend(){
    CAN_TxHeaderTypeDef txHeaderTypeDef;

    txHeaderTypeDef.StdId = 0x101;
    txHeaderTypeDef.DLC = 0x08;
    txHeaderTypeDef.IDE = CAN_ID_STD;
    txHeaderTypeDef.RTR = CAN_RTR_DATA;
    txHeaderTypeDef.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(&hcan2, &txHeaderTypeDef,arm2message,0);
}

void ARMMotor::ARMStop(){
    arm1message[0] = 0xFD;
    arm1message[1] = 0x12;//1代表正转，0代表反转
    arm1message[2] = 0xFF;//4FF代表速度
    arm1message[3] = 0x00;//加速度，0为不启用,若需较快，改为0x96
    arm1message[4] = 0x00;
    arm1message[5] = 0x00;
    arm1message[6] = 0x00;//4~6为脉冲数,3200为一圈，对应z轴8mm
    arm1message[7] = 0x6B;

    arm2message[0] = 0x00;//位置低字节
    arm2message[1] = 0x00;
    arm2message[2] = 0x00;
    arm2message[3] = 0x00;//位置高字节
    arm2message[4] = 0x00;//速度低字节
    arm2message[5] = 0x00;
    arm2message[6] = 0x00 ;
    arm2message[7] = 0x00;//速度高字节


}
/**
 * @brief 统一发送函数
 */
void ARMMotor::PackageSend() {
    if(Arm1SendFlag == 1){
        ARMCAN1PackageSend();
        Arm1SendFlag = 0;
    }
   // ARMCAN1PackageSend();
    ARMCAN2PackageSend();
}
/*
*//**
 * @brief 用于设置电机速度
 * @param _targetSpeed 目标速度
 *//*
void ARMMotor::SetTargetSpeed(float _targetSpeed) {
    stopFlag = 0;
    targetSpeed = _targetSpeed;
}
*//**
 * @brief 用于设置电机角度
 * @param _targetAngle 目标角度
 *//*
void ARMMotor::SetTargetAngle(float _targetAngle) {
    stopFlag = false;
    targetAngle = _targetAngle;
}
int16_t ARMMotor::AngleGenerate(){
    int16_t Angle;
    switch (motorType) {
        case ARM1:
            Angle = CatchControl::cc_ctrl.ARM1.angle;
            break;
        case ARM2:
            Angle = CatchControl::cc_ctrl.ARM2.angle;
            break;
        case ARM3:
            Angle = CatchControl::cc_ctrl.ARM3.angle;
            break;
    }
    return Angle;
}
int16_t ARMMotor::SpeedGenerate(){
    int16_t Speed;
    switch (motorType) {
        case ARM1:
            Speed = CatchControl::cc_ctrl.ARM1.speed;
            break;
        case ARM2:
            Speed = CatchControl::cc_ctrl.ARM2.speed;
            break;
        case ARM3:
            Speed = CatchControl::cc_ctrl.ARM3.speed;
            break;

    }
    return Speed;
}*/
/**
* @brief ARMMotor类的构造函数
 * @param
*/
ARMMotor::ARMMotor(MOTOR_TYPE_e* MotorType){
    motorType = *MotorType;

}
/**
* @brief ARMMotor类的析构函数
 * @param
*/
ARMMotor::~ARMMotor(){

}

void ARMMotor::Handle() {


    if (stopFlag == 1) {
        ARMStop();
    } else {
        switch (motorType) {

            case ARM1:
                ARMCAN1MessageGenerate();
                break;

            case ARM2:
                ARMCAN2MessageGenerate();
                break;

        }
    }
}
/**
 * @brief 电机的中断处理函数，用于电调返回数据的接受
 * @param hcan
 */
void ARMMotor::IT_Handle(CAN_HandleTypeDef *hcan) {
    uint8_t canBuf[8];
    CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, canBuf);
    if(hcan == &hcan2 && rx_header.StdId == 0x01){
        if(canBuf[1] == 0x9F){
            Arm1SendFlag = 1;
        }
    }

}

void ARMMotor::Stop() {
    stopFlag = true;
}
void ARMMotor::ErrorHandle() {

}

TRAYMotor::TRAYMotor(){

}
TRAYMotor::~TRAYMotor(){

}
void TRAYMotor::Handle() {
    TRAYFlagGenerate();
}
void TRAYMotor::ErrorHandle() {

}
/**
 * @brief 托盘电机消息获取
 */
void TRAYMotor::TRAYFlagGenerate(){
    trayflag = CatchControl::cc_ctrl.TrayFlag;
}
/**
 * @brief 托盘电机发送任务
 */
void TRAYMotor::TrayPackageSend(){
    CAN_TxHeaderTypeDef txHeaderTypeDef;

    txHeaderTypeDef.StdId = 0x142;
    txHeaderTypeDef.DLC = 0x08;
    txHeaderTypeDef.IDE = CAN_ID_STD;
    txHeaderTypeDef.RTR = CAN_RTR_DATA;
    txHeaderTypeDef.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(&hcan2, &txHeaderTypeDef,traymessage[trayflag],0);
}