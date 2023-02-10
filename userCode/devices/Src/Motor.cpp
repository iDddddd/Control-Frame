//
// Created by LEGION on 2021/10/4.
//

#include "Motor.h"

uint8_t RS485::rsmessage[4][11];
int16_t Motor_4315::motor4315_intensity[8];

/*Motor类----------------------------------------------------------------*/
/**
 * @brief Motor类的构造函数
 * @param _init 类的初始化结构体指针
 */
Motor::Motor(MOTOR_INIT_t *_init) {
    deviceType = MOTOR;

    if(_init->speedPIDp) speedPID.PIDInfo = *_init->speedPIDp;
    if(_init->anglePIDp) anglePID.PIDInfo = *_init->anglePIDp;
    reductionRatio = _init->reductionRatio;
    commuType = _init->commuType;
}
/**
* @brief Motor类的析构函数
*/
Motor::~Motor(){}
void Motor::Handle(){}
void Motor::ErrorHandle(){}
/*RS485类------------------------------------------------------------------*/
/**
 * @brief RS485类的构造函数
 */
RS485::RS485(uint16_t _id){
    motor_ID = GET_MOTOR_POS(_id);
}
/**
 * @brief RS485类的析构函数
 */
RS485::~RS485(){

}
/**
 * @brief RS485消息包发送任务
 */
void RS485::RS485PackageSend(){
    static uint8_t rsmotorIndex = 0;
    rsmotorIndex %= 4;
    HAL_UART_Transmit_IT(&huart1, rsmessage[rsmotorIndex], 11);

    rsmotorIndex++;
}

/*4315电机类------------------------------------------------------------------*/
/**
 * @brief Motor_4315类的构造函数
 */
Motor_4315::Motor_4315(uint16_t _id,MOTOR_INIT_t* _init) : Motor(_init), RS485(_id){

}

/**
 * @brief Motor_4315类的析构函数
 */
Motor_4315::~Motor_4315(){

}
/**
 * @brief 4315电机消息包获取任务
 */
void Motor_4315::RS485MessageGenerate() {
    int motorIndex = motor_ID;

    rsmessage[motorIndex][0] = 0x3E;//协议头
    rsmessage[motorIndex][1] = 0x00;//包序号
    rsmessage[motorIndex][2] = 0x01 + motorIndex; //ID
    rsmessage[motorIndex][3] = 0x55;//相对位置闭环控制命令码
    rsmessage[motorIndex][4] = 0x04;//数据包长度

    uint32_t tmp = motor4315_intensity[motorIndex] ;
    rsmessage[motorIndex][8] = tmp >> 24u;
    rsmessage[motorIndex][7] = tmp >> 16u;
    rsmessage[motorIndex][6] = tmp >> 8u;
    rsmessage[motorIndex][5] = tmp;

    uint16_t crc = CRC16Calc(rsmessage[motorIndex], 9);
    rsmessage[motorIndex][9] = crc;
    rsmessage[motorIndex][10] = crc >> 8u;
}
/**
 * @brief 4315电机类的执行处理函数
 */
void Motor_4315::Handle(){

    int16_t intensity = targetAngle;

    if (stopFlag == 1){
        motor4315_intensity[motor_ID] = 0;
    }else {
        motor4315_intensity[motor_ID] = intensity;
    }

    RS485MessageGenerate();
}

/**
 * @brief 用于设置4315电机角度
 * @param _targetAngle 目标角度
 */
void Motor_4315::SetTargetAngle(float _targetAngle){
    stopFlag = 0;
    targetAngle = _targetAngle * 16384 / 360.0f;
}

/**
 * @brief 用于使4315电机停止
 */
void Motor_4315::Stop() {
    stopFlag = true;
}

uint16_t Motor_4315::CRC16Calc(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xffff;        // Initial value
    while (length--) {
        crc ^= *data++;            // crc ^= *data; data++;
        for (uint8_t i = 0; i < 8; ++i) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;        // 0xA001 = reverse 0x8005
            else
                crc = (crc >> 1);
        }
    }
    return crc;
}





/*
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    Motor::IT_Handle(hcan);
}*/


