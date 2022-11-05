//
// Created by LEGION on 2021/10/4.
//

#include "Motor.h"
#include <queue>

Motor* Motor::motorPtrs[2][8] = {0};
int16_t Motor::motor_intensity[16];
uint32_t Motor::motor_angle[16];
uint32_t Motor::motor_IDs;

/**
 * 计算pid算法的控制量
 * @param target 目标量
 * @param feedback 反馈量
 * @return 控制量
 */
float PID::PIDCalc(float target,float feedback) {
    PIDInfo.fdb = feedback;
    PIDInfo.ref = target;
    PIDInfo.err[3] = PIDInfo.ref - PIDInfo.fdb;
    PIDInfo.componentKp = PIDInfo.err[3] * PIDInfo.kp;
    PIDInfo.errSum += PIDInfo.err[3];
    INRANGE(PIDInfo.errSum, -1 * PIDInfo.componentKiMax / PIDInfo.ki, PIDInfo.componentKiMax / PIDInfo.ki);
    PIDInfo.componentKi = PIDInfo.errSum * PIDInfo.ki;
    PIDInfo.componentKd = (PIDInfo.err[3] - PIDInfo.err[2]) * PIDInfo.kd;
    INRANGE(PIDInfo.componentKp, -1 * PIDInfo.componentKpMax, PIDInfo.componentKpMax);
    INRANGE(PIDInfo.componentKi, -1 * PIDInfo.componentKiMax, PIDInfo.componentKiMax);
    INRANGE(PIDInfo.componentKd, -1 * PIDInfo.componentKdMax, PIDInfo.componentKdMax);
    PIDInfo.output = PIDInfo.componentKp + PIDInfo.componentKi + PIDInfo.componentKd;
    INRANGE(PIDInfo.output, -1 * PIDInfo.outputMax, PIDInfo.outputMax);
    PIDInfo.err[2] = PIDInfo.err[3];
    return PIDInfo.output;
}


/**
 * @brief 电机类的初始化，主要是CAN通信的相关配置
 * @callergraph int main(void) in Device.cpp
 */
void Motor::Init(){
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

    CAN_FilterTypeDef canFilterTypeDef;

    canFilterTypeDef.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterTypeDef.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilterTypeDef.FilterIdHigh = 0x0000;
    canFilterTypeDef.FilterIdLow = 0x0000;
    canFilterTypeDef.FilterMaskIdHigh = 0x0000;
    canFilterTypeDef.FilterMaskIdLow = 0x0000;
    canFilterTypeDef.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilterTypeDef.FilterActivation = ENABLE;
    canFilterTypeDef.FilterBank = 0;
    canFilterTypeDef.SlaveStartFilterBank = 0;

    HAL_CAN_ConfigFilter(&hcan1, &canFilterTypeDef);
    HAL_CAN_ConfigFilter(&hcan2, &canFilterTypeDef);
}
/**
 * @brief CRC16_MODBUS校验
 *
 */
uint16_t Motor::crc16_modbus(uint8_t *data, uint16_t length)
{
    uint8_t i;
    uint16_t crc = 0xffff;        // Initial value
    while(length--)
    {
        crc ^= *data++;            // crc ^= *data; data++;
        for (i = 0; i < 8; ++i)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;        // 0xA001 = reverse 0x8005
            else
                crc = (crc >> 1);
        }
    }
    return crc;
}
/**
 * @brief rs485消息包发送任务
 */
void Motor::RS485PackageSend(){
    static uint8_t rs485Buf[11] = {0};
    uint16_t crc;
    rs485Buf[0] = 0x3E;//协议头
    rs485Buf[1] = 0x00;//包序号

    rs485Buf[3] = 0x55;//绝对位置闭环控制命令码
    rs485Buf[4] = 0x04;//数据包长度
    for (int motorIndex = 0; motorIndex < 4; motorIndex++ ){
        if((1 << motorIndex) & motor_IDs) {
            rs485Buf[2] = 0x01 + motorIndex;//设备地址
            rs485Buf[5] = motor_angle[motorIndex] >> 24u;//目标位置低字节
            rs485Buf[6] = motor_angle[motorIndex] >> 16u;
            rs485Buf[7] = motor_angle[motorIndex] >> 8u;
            rs485Buf[8] = motor_angle[motorIndex];//目标位置高字节
        }
    }
    crc = crc16_modbus(rs485Buf,9);
    rs485Buf[9] = crc;//CRC校验低位
    rs485Buf[10] = crc >> 8u;//CRC校验高位
    for (int motorIndex = 0; motorIndex < 4; motorIndex++ ){
				rs485Buf[2] = 0x01 ;
        //HAL_UART_Transmit(&huart1, rs485Buf, 11, 50);
        HAL_UART_Transmit_DMA(&huart1, rs485Buf, 11);
        HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_10);
    }
}


/**
 * @brief can消息包发送任务
 * @callergraph void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 *              in Device.cpp
 */
void Motor::CANPackageSend() {
    uint8_t canBuf[8] = {0};

    CAN_TxHeaderTypeDef txHeaderTypeDef;


    txHeaderTypeDef.DLC = 0x08;
    txHeaderTypeDef.IDE = CAN_ID_STD;
    txHeaderTypeDef.RTR = CAN_RTR_DATA;
    txHeaderTypeDef.TransmitGlobalTime = DISABLE;

    for(auto motorIndex = 0; motorIndex < 4; motorIndex++) {
        if((1 << motorIndex) & motor_IDs){
            txHeaderTypeDef.StdId = 0x144 - motorIndex;
				
            canBuf[0] = 0xA1;
            canBuf[4] = motor_intensity[motorIndex] >> 8u;
            canBuf[5] = motor_intensity[motorIndex];
						while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0)
							;
            HAL_CAN_AddTxMessage(&hcan1,&txHeaderTypeDef,canBuf,0);
        }
    }
}
/**
 * @brief Motor类的构造函数
 * @param _init 类的初始化结构体指针
 */
Motor::Motor(MOTOR_INIT_t* _init){

    deviceID = _init->_motorID;
    deviceType = MOTOR;

    auto motorPos = GET_MOTOR_POS(_init->_motorID);
    motorPtrs[motorPos/8][motorPos%8] = this;
    //TODO 检查电机ID冲突
    motor_IDs |= _init->_motorID;
    memset(&feedback,0,sizeof(C6x0Rx_t));

    if(_init->speedPIDp) speedPID.PIDInfo = *_init->speedPIDp;
    if(_init->anglePIDp) anglePID.PIDInfo = *_init->anglePIDp;

    ctrlType = _init->ctrlType;
    reductionRatio = _init->reductionRatio;
    targetSpeed = 0;
    rsTargetAngle = 0;
}
/**
 * @brief Motor类的构造函数另一重载，可以方便地定义参数相同，ID不同的电机
 * @param _id 电机ID
 * @param _init 电机初始化结构体指针
 */
Motor::Motor(uint32_t _id, MOTOR_INIT_t* _init) {
    _init->_motorID = _id;

    deviceID = _init->_motorID;
    deviceType = MOTOR;

    auto motorPos = GET_MOTOR_POS(_init->_motorID);
    motorPtrs[motorPos/8][motorPos%8] = this;
    //TODO 检查电机ID冲突
    motor_IDs |= _init->_motorID;
    memset(&feedback,0,sizeof(C6x0Rx_t));

    if(_init->speedPIDp) speedPID.PIDInfo = *_init->speedPIDp;
    if(_init->anglePIDp) anglePID.PIDInfo = *_init->anglePIDp;

    ctrlType = _init->ctrlType;
    reductionRatio = _init->reductionRatio;
    targetSpeed = 0;
    rsTargetAngle = 0;
}
/**
 * @brief 电机类的析构函数
 */
Motor::~Motor(){
    motor_IDs &= (~deviceID);
}
/**
 * @brief 电机类的执行处理函数，囊括每个电机的主要处理任务。此函数需定时执行，否则对应电机无法正常工作
 */
void Motor::Handle(){
    MotorStateUpdate();

    int16_t intensity = IntensityCalc();

    if (stopFlag == 1){
        motor_intensity[GET_MOTOR_POS(deviceID)] = 0;
        motor_angle[GET_MOTOR_POS(deviceID)] = 0;
    }else {
        motor_intensity[GET_MOTOR_POS(deviceID)] = intensity;
        motor_angle[GET_MOTOR_POS(deviceID)] = rsTargetAngle;
    }
}


/**
 * @brief 电机的中断处理函数，用于电调返回数据的接受
 * @param hcan
 */
void Motor::IT_Handle(CAN_HandleTypeDef *hcan) {
    uint8_t canBuf[8];
    CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, canBuf);
    uint8_t canPos,motorPos;
    if(hcan == &hcan1){
        canPos = 0;
    }else {
        canPos = 1;
    }
    motorPos = rx_header.StdId - 0x141;
    motorPtrs[canPos][motorPos]->feedback.angle = canBuf[6] | (canBuf[7]<<8u);
    motorPtrs[canPos][motorPos]->feedback.speed = canBuf[4] | (canBuf[5]<<8u);
    motorPtrs[canPos][motorPos]->feedback.moment = canBuf[2] | (canBuf[3]<<8u);
    motorPtrs[canPos][motorPos]->feedback.temp = canBuf[1];
}
/**
 * @brief 用于设置4315电机角度
 */
void Motor::RSTargetAngle(float _rsTargetAngle){
    stopFlag = 0;
    rsTargetAngle = _rsTargetAngle;
}
/**
 * @brief 用于设置4010电机速度，只用当电机控制类型对应时才有效
 * @param _targetSpeed 目标速度
 */
void Motor::SetTargetSpeed(float _targetSpeed) {
    stopFlag = 0;
    targetSpeed = _targetSpeed;
}
/**
 * @brief 用于设置4010电机角度，只用当电机控制类型对应时才有效
 * @param _targetAngle 目标角度
 */
void Motor::SetTargetAngle(float _targetAngle) {
    stopFlag = 0;
    targetAngle = _targetAngle;
}

void Motor::Stop() {
    stopFlag = 1;
}

void Motor::ErrorHandle() {

}
/**
 * @brief 更新电机的相关状态
 * @callergraph this->Handle()
 */
void Motor::MotorStateUpdate() {
    state.speed = feedback.speed/reductionRatio;
    state.moment = feedback.moment;
    state.temperature = feedback.temp;

    float realAngle = state.angle;
    float thisAngle = feedback.angle;
    static int32_t lastRead = 0;
    if (thisAngle <= lastRead) {
        if (lastRead - thisAngle > 8000)
            realAngle += (thisAngle + 16384 - lastRead) * 360.0f / 16384.0f / reductionRatio;
        else
            realAngle -= (lastRead - thisAngle) * 360.0f / 16384.0f / reductionRatio;
    } else {
        if (thisAngle - lastRead > 8000)
            realAngle -= (lastRead + 16384 - thisAngle) * 360.0f / 16384.0f / reductionRatio;
        else
            realAngle += (thisAngle - lastRead) * 360.0f / 16384.0f / reductionRatio;
    }

    state.angle = realAngle;
    lastRead = feedback.angle;
}
/**
 * @brief 计算电机实际控制电流
 * @return 控制电流值
 */
int16_t Motor::IntensityCalc(){
    int16_t intensity;
    switch (ctrlType) {
        case SPEED_Single:
            intensity = speedPID.PIDCalc(targetSpeed, state.speed);
            break;
        case POSITION_Double:
            float _targetSpeed = anglePID.PIDCalc(targetAngle,state.angle);
            intensity = speedPID.PIDCalc(_targetSpeed, state.speed);
            break;
    }
    return intensity;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    Motor::IT_Handle(hcan);
}
