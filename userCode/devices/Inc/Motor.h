//
// Created by LEGION on 2021/10/4.
//

#ifndef RM_FRAME_C_MOTOR_H
#define RM_FRAME_C_MOTOR_H

#include "Device.h"
#include "can.h"
#include "PID.h"
#include <cstring>



#define GET_MOTOR_POS(ID)  (uint32_t)(log2(ID))
/*枚举类型定义------------------------------------------------------------*/
/**
 * @enum 控制电机的方式
 * @example SPEED_Single 单环电机，控制速度
 * @example POSITION_Double 双环电机，控制角度
 */
typedef enum {
    DIRECT = 0,
    SPEED_Single,
    POSITION_Double
} MOTOR_CTRL_TYPE_e;

typedef enum {
    CAN = 0,
    RS485,
} MOTOR_COMMU_TYPE_e;

/*结构体定义--------------------------------------------------------------*/
typedef struct {
    uint16_t angle;
    int16_t speed;
    int16_t moment;
    int8_t temp;
} C6x0Rx_t;

typedef struct {

    PID_Regulator_t* speedPIDp;//速度环pid参数结构体指针
    PID_Regulator_t* anglePIDp;//角度环pid参数结构体指针
    float reductionRatio;//减速比
    MOTOR_COMMU_TYPE_e commuType;

} MOTOR_INIT_t;

typedef struct {

    float speed;//最终电机输出轴的转速，单位为RPM
    float angle;//输出轴的角度，单位为度
    float moment;//转矩电流的相对值，具体值参考电调手册
    float temperature;//电机温度，单位摄氏度

}MOTOR_STATE_t;
/*类型定义----------------------------------------------------------------*/

/*Motor类----------------------------------------------------------------*/
class Motor :private Device
{
public:
    explicit Motor(MOTOR_INIT_t* _init);
    ~Motor();
    void Handle() override;
    void ErrorHandle() override;

protected:
    PID speedPID,anglePID;
    MOTOR_COMMU_TYPE_e commuType;
    float reductionRatio;

};

/*RS485类------------------------------------------------------------------*/
class RS485
{
public:
    uint16_t motor_ID;
    static uint8_t rsmessage[4][11];

    explicit RS485(uint16_t _id);
    ~RS485();

    static void RS485PackageSend();
    virtual void RS485MessageGenerate() = 0;

protected:
    MOTOR_CTRL_TYPE_e ctrlType;

};

/*4315电机类------------------------------------------------------------------*/
class Motor_4315:public Motor, public RS485{
public:
    static int16_t motor4315_intensity[8];
    bool stopFlag{true};
    float targetAngle = 0;

    void RS485MessageGenerate() override;
    void Handle() override;

    void SetTargetAngle(float _targetAngle);
    void Stop();

    Motor_4315(uint16_t _id,MOTOR_INIT_t* _init);
    ~Motor_4315();

private:
    uint16_t CRC16Calc(uint8_t *data, uint16_t length);

};
/*结构体成员取值定义组------------------------------------------------------*/

/**
 * @defgroup motor_IDs
 * @brief 电机ID前八个对应C型开发板can1上1到8的ID，9到16对应C型开发板can2上的1到8
 */
#define MOTOR_ID_1 0x00000001u
#define MOTOR_ID_2 0x00000002u
#define MOTOR_ID_3 0x00000004u
#define MOTOR_ID_4 0x00000008u
#define MOTOR_ID_5 0x00000016u
#define MOTOR_ID_6 0x00000020u
#define MOTOR_ID_7 0x00000040u
#define MOTOR_ID_8 0x00000080u

#define MOTOR_ID_9  0x00000100u
#define MOTOR_ID_10 0x00000200u
#define MOTOR_ID_11 0x00000400u
#define MOTOR_ID_12 0x00000800u
#define MOTOR_ID_13 0x00001000u
#define MOTOR_ID_14 0x00002000u
#define MOTOR_ID_15 0x00004000u
#define MOTOR_ID_16 0x00008000u
#define MOTOR_ID_CAN1_1_MASK MOTOR_ID_1|MOTOR_ID_2|MOTOR_ID_3|MOTOR_ID_4
#define MOTOR_ID_CAN1_2_MASK MOTOR_ID_5|MOTOR_ID_6|MOTOR_ID_7|MOTOR_ID_8
#define MOTOR_ID_CAN2_1_MASK MOTOR_ID_9|MOTOR_ID_10|MOTOR_ID_11|MOTOR_ID_12
#define MOTOR_ID_CAN2_2_MASK MOTOR_ID_13|MOTOR_ID_14|MOTOR_ID_15|MOTOR_ID_16

/*外部变量声明-------------------------------------------------------------*/
/*外部函数声明-------------------------------------------------------------*/



#endif //RM_FRAME_C_MOTOR_H
