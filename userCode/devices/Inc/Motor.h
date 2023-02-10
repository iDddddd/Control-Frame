//
// Created by LEGION on 2021/10/4.
//

#ifndef RM_FRAME_C_MOTOR_H
#define RM_FRAME_C_MOTOR_H

#include "Device.h"
#include "can.h"
#include "PID.h"
#include <cstring>
#include <map>


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
} MOTOR_INIT_t;

typedef struct {
    uint16_t _id;//canID
    MOTOR_CTRL_TYPE_e ctrlType;
}COMMU_INIT_t;

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
    float reductionRatio;

};
/*CAN类------------------------------------------------------------------*/
class CAN{
public:
    static CAN* canPtrs[8];
    uint16_t can_ID;
    static uint8_t canmessage[8];

    static void CANInit();

    CAN(COMMU_INIT_t* _init,uint8_t* RxMessage);
    ~CAN();

    static void CANPackageSend();
    static void Rx_Handle(CAN_HandleTypeDef *hcan);
    virtual void CANMessageGenerate() = 0;

protected:
    MOTOR_STATE_t state{};
    MOTOR_CTRL_TYPE_e ctrlType;
    static std::map<uint16_t,uint8_t*> dict;

};
/*RS485类------------------------------------------------------------------*/
class RS485
{
public:
    uint16_t rs485_ID;
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

/*4010电机类------------------------------------------------------------------*/
class Motor_4010:public Motor, public CAN{
public:
    uint8_t RxMessage[8]{};
    static int16_t motor4010_intensity[8];

    C6x0Rx_t feedback{};
    bool stopFlag{true};
    float targetSpeed = 0;
    float targetAngle = 0;

    void CANMessageGenerate() override;
    void Handle() override;

    void SetTargetSpeed(float _targetSpeed);
    void SetTargetAngle(float _targetAngle);
    void Stop();

    Motor_4010(COMMU_INIT_t* commu_init,MOTOR_INIT_t* motor_init);
    ~Motor_4010();
private:

    void MotorStateUpdate();
    int16_t IntensityCalc();
};
/*结构体成员取值定义组------------------------------------------------------*/

/**
 * @defgroup motor_IDs
 * @brief 电机ID前八个对应C型开发板can1上1到8的ID，9到16对应C型开发板can2上的1到8
 */
#define MOTOR_ID_1 0
#define MOTOR_ID_2 1
#define MOTOR_ID_3 2
#define MOTOR_ID_4 3
#define MOTOR_ID_5 4
#define MOTOR_ID_6 5
#define MOTOR_ID_7 6
#define MOTOR_ID_8 7

/*外部变量声明-------------------------------------------------------------*/
/*外部函数声明-------------------------------------------------------------*/



#endif //RM_FRAME_C_MOTOR_H
