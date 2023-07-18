//
// Created by LEGION on 2021/10/4.
//

#include "Motor.h"


Motor_Object_t *Motor::head_;

/*Motor类----------------------------------------------------------------*/
/**
 * @brief Motor类的构造函数
 * @param _init 类的初始化结构体指针
 */
Motor::Motor(MOTOR_INIT_t *_init, Motor *motor) {
    deviceType = MOTOR;//设备类型为电机

    if (_init->speedPIDp) speedPID.PIDInfo = *_init->speedPIDp;//如果速度环pid参数结构体指针不为空，则将参数结构体赋值给类的参数结构体
    if (_init->anglePIDp) anglePID.PIDInfo = *_init->anglePIDp;//如果角度环pid参数结构体指针不为空，则将参数结构体赋值给类的参数结构体
    ctrlType = _init->ctrlType;//控制电机的方式
    reductionRatio = _init->reductionRatio;//减速比
    //将电机对象加入电机链表，实现后续中断处理
    auto *new_object = new Motor_Object_t();
    new_object->motor_object = motor;
    new_object->next = head_;
    head_ = new_object;

}

/**
* @brief Motor类的析构函数
*/
Motor::~Motor() = default;

void Motor::ErrorHandle() {}

void Motor::MotorsHandle() {

    Motor_Object_t *current = head_;
    while (current) {
        current->motor_object->Handle();//调用每个电机的中断处理函数
        current = current->next;
    }

}

void Motor::Stop() {
    stopFlag = true;

}


/*4010电机类------------------------------------------------------------------*/
/**
 * @brief Motor_4010类的构造函数
 */
FOUR_Motor_4010::FOUR_Motor_4010(COMMU_INIT_t *commu_init1, COMMU_INIT_t *commu_init2,
                                 COMMU_INIT_t *commu_init3, COMMU_INIT_t *commu_init4, MOTOR_INIT_t *motor_init1,
                                 MOTOR_INIT_t *motor_init2,MOTOR_INIT_t *motor_init3,MOTOR_INIT_t *motor_init4)
        : Motor(motor_init1, this) {
    canIDs[0] = commu_init1->_id;
    canIDs[1] = commu_init2->_id;
    canIDs[2] = commu_init3->_id;
    canIDs[3] = commu_init4->_id;
    if (motor_init1->speedPIDp) speedPIDs[0].PIDInfo = *motor_init1->speedPIDp;
    if (motor_init2->speedPIDp) speedPIDs[1].PIDInfo = *motor_init2->speedPIDp;
    if (motor_init3->speedPIDp) speedPIDs[2].PIDInfo = *motor_init3->speedPIDp;
    if (motor_init4->speedPIDp) speedPIDs[3].PIDInfo = *motor_init4->speedPIDp;
    FOURID_Bind_Rx(canIDs, RxMessage);

}

/**
 * @brief Motor_4010类的析构函数
 */
FOUR_Motor_4010::~FOUR_Motor_4010() = default;

/**
 * @brief 4010电机类的执行处理函数
 */
void FOUR_Motor_4010::Handle() {
    int16_t intensity[4];
    uint32_t id;

    for (auto canID: canIDs) {
        id = canID - 0x141;//将电机id与数组下标对应
        MotorStateUpdate(id);//更新电机状态
        intensity[id] = IntensityCalc(id);//计算电机转矩值
        if (stopFlag) {//如果停止标志位为真，则将电机转矩值置为0
            motor4010_intensity[id] = 0;
        } else {
            motor4010_intensity[id] = intensity[id];
        }
    }
    CANMessageGenerate();//生成can消息包
}

/**
 * @brief 4010电机类的消息包获取任务
 */
void FOUR_Motor_4010::CANMessageGenerate() {
    if ((canQueue.rear + 1) % MAX_MESSAGE_COUNT != canQueue.front) {//如果队列未满

        canQueue.Data[canQueue.rear].ID = can_ID;//can_ID用于记录当前发送的canID，此处由于同时控四个电机，根据电机应为0x280
        canQueue.Data[canQueue.rear].canType = canType;//canType用于记录当前发送的can类型
        canQueue.Data[canQueue.rear].message[0] = motor4010_intensity[0];//第一个电机转矩值，对应id为0x141
        canQueue.Data[canQueue.rear].message[1] = motor4010_intensity[0] >> 8u;
        canQueue.Data[canQueue.rear].message[2] = motor4010_intensity[1];//第二个电机转矩值，对应id为0x142
        canQueue.Data[canQueue.rear].message[3] = motor4010_intensity[1] >> 8u;
        canQueue.Data[canQueue.rear].message[4] = motor4010_intensity[2];//第三个电机转矩值，对应id为0x143
        canQueue.Data[canQueue.rear].message[5] = motor4010_intensity[2] >> 8u;
        canQueue.Data[canQueue.rear].message[6] = motor4010_intensity[3];//第四个电机转矩值，对应id为0x144
        canQueue.Data[canQueue.rear].message[7] = motor4010_intensity[3] >> 8u;

        canQueue.rear = (canQueue.rear + 1) % MAX_MESSAGE_COUNT;//队尾指针后移
    } else{//如果队列已满,则清空队列
        canQueue.rear = 0;
        canQueue.front = 0;
    }
}

/**
 * @brief 用于设置4010电机速度
 * @param _targetSpeed 目标速度
 */
void FOUR_Motor_4010::SetTargetSpeed(const float *_targetSpeed) {
    stopFlag = false;//停止标志位为假
    targetSpeed[0] = _targetSpeed[0];//将目标速度赋值给目标速度数组
    targetSpeed[1] = _targetSpeed[1];
    targetSpeed[2] = _targetSpeed[2];
    targetSpeed[3] = _targetSpeed[3];

}


/**
 * @brief 更新电机的相关状态
 * @callergraph this->Handle()
 */
void FOUR_Motor_4010::MotorStateUpdate(uint32_t id) {
    //接收电机反馈信息并处理
    feedback[id].angle = RxMessage[id][6] | (RxMessage[id][7] << 8u);
    feedback[id].speed = RxMessage[id][4] | (RxMessage[id][5] << 8u);
    feedback[id].moment = RxMessage[id][2] | (RxMessage[id][3] << 8u);
    feedback[id].temp = RxMessage[id][1];

    switch (ctrlType) {
        case SPEED_Single: {//电机速度控制
            //直接将电机反馈值赋值给state
            state[id].speed = feedback[id].speed / reductionRatio;
        }
        case POSITION_Double: {//电机位置控制
            //此处代码是为了使电机在位置控制模式下可以跨越0-360度的位置
            state[id].speed = feedback[id].speed / reductionRatio;
            state[id].moment = feedback[id].moment;
            state[id].temperature = feedback[id].temp;
            state[id].angle = feedback[id].angle * 360 / 16384;
            float realAngle = state[id].angle;
            float thisAngle = feedback[id].angle;
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
            state[id].angle = realAngle;
            lastRead = feedback[id].angle;
            break;
        }
        case DIRECT://电机内部控制

            break;
    }

}

/**
 * @brief 计算电机实际控制电流
 * @return 控制电流值
 */
int16_t FOUR_Motor_4010::IntensityCalc(uint32_t id) {
    int16_t intensity = 0;

    switch (ctrlType) {
        case DIRECT://电机内部控制
            intensity = targetSpeed[id] * 16384 / 360.0f;
            break;

        case SPEED_Single://电机速度控制
            intensity = speedPIDs[id].PIDCalc(targetSpeed[id], state[id].speed);
            break;

        case POSITION_Double://电机位置控制
        //目前不做多电机位置位置，此处缺少参数targetAngle，故注释
            /*float _targetSpeed = anglePID.PIDCalc(targetAngle, state.angle);
            intensity = (int16_t) speedPID.PIDCalc(_targetSpeed, state.speed);*/
            break;
    }
    return intensity;
}


/*4315电机类------------------------------------------------------------------*/
/**
 * @brief Motor_4315类的构造函数
 */
Motor_4315::Motor_4315(uint32_t _id, MOTOR_INIT_t *_init) : Motor(_init, this), RS485(_id) {
    ID_Bind_Rx(RxMessage);
}

/**
 * @brief Motor_4315类的析构函数
 */
Motor_4315::~Motor_4315() = default;

/**
 * @brief 4315电机消息包获取任务
 */
void Motor_4315::RS485MessageGenerate() {
    int motorIndex = rs485_ID;

    rsmessage[motorIndex][0] = 0x3E;//协议头
    rsmessage[motorIndex][1] = 0x00;//包序号
    rsmessage[motorIndex][2] = 0x01 + motorIndex; //ID
    rsmessage[motorIndex][3] = 0x55;//相对位置闭环控制命令码
    rsmessage[motorIndex][4] = 0x04;//数据包长度

    rsmessage[motorIndex][5] = motor4315_angle[motorIndex];
    rsmessage[motorIndex][6] = motor4315_angle[motorIndex] >> 8u;
    rsmessage[motorIndex][7] = motor4315_angle[motorIndex] >> 16u;
    rsmessage[motorIndex][8] = motor4315_angle[motorIndex] >> 24u;

    uint16_t crc = CRC16Calc(rsmessage[motorIndex], 9);
    rsmessage[motorIndex][9] = crc;
    rsmessage[motorIndex][10] = crc >> 8u;
}

/**
 * @brief 4315电机类的执行处理函数
 */
void Motor_4315::Handle() {
    nowAngle = (RxMessage[7] | (RxMessage[8] << 8u) | (RxMessage[9] << 16u) | (RxMessage[10] << 24u) ) / 16384.0f * 360.0f;

    AngleCalc();
    if (stopFlag == 1) {
        motor4315_angle[rs485_ID] = (int32_t )(zeroAngle * 16384.0f / 360.0f);
    } else {
        motor4315_angle[rs485_ID] = (int32_t )(realAngle * 16384.0f / 360.0f);
    }

    RS485MessageGenerate();

}

/**
 * @brief 用于设置4315电机角度
 * @param _targetAngle 目标角度
 */
void Motor_4315::SetTargetAngle(float _targetAngle) {
    stopFlag = false;
    targetAngle = _targetAngle;
}

void Motor_4315::AngleCalc() {
    if (targetAngle - lastAngle > 180) {
        zeroAngle -= 360;
    }
    if (lastAngle - targetAngle > 180) {
        zeroAngle += 360;
    }
    lastAngle = targetAngle;
    realAngle = targetAngle + zeroAngle;
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


