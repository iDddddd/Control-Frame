//
// Created by LEGION on 2021/10/4.
//

#ifndef RM_FRAME_C_MOTOR_H
#define RM_FRAME_C_MOTOR_H

#include "Device.h"
#include "can.h"
#include "PID.h"
#include "CommuType.h"
#include <cstring>

/*枚举类型定义------------------------------------------------------------*/
/**
 * @enum 控制电机的方式
 * @example SPEED_Single 单环电机，控制速度
 * @example POSITION_Double 双环电机，控制角度
 */

/*结构体定义--------------------------------------------------------------*/
typedef enum {
    DIRECT = 0,//使用电机内部PID
    SPEED_Single,//单环电机，控制速度,需要一个pid参数
    POSITION_Double//双环电机，控制角度，需要两个pid参数
} MOTOR_CTRL_TYPE_e;

typedef struct {
    uint16_t angle;
    int16_t speed;
    int16_t moment;
    int8_t temp;
} MOTOR_FEEDBACK_t;//电机反馈数据结构体

typedef struct {
    PID_Regulator_t *speedPIDp;//速度环pid参数结构体指针
    PID_Regulator_t *anglePIDp;//角度环pid参数结构体指针
    MOTOR_CTRL_TYPE_e ctrlType;//控制电机的方式
    float reductionRatio;//减速比
} MOTOR_INIT_t;


typedef struct {

    float speed;//最终电机输出轴的转速，单位为RPM
    float angle;//输出轴的角度，单位为度
    float moment;//转矩电流的相对值，具体值参考电调手册
    float temperature;//电机温度，单位摄氏度

} MOTOR_STATE_t;

class Motor;

struct Motor_Object_t {
    Motor *motor_object;
    Motor_Object_t *next;
};//电机对象结构体

/*类型定义----------------------------------------------------------------*/

/*Motor类----------------------------------------------------------------*/
/**
 * @class Motor类
 */
class Motor : private Device {
public:
    Motor(MOTOR_INIT_t *_init, Motor *motor);//电机初始化函数

    ~Motor();

    void ErrorHandle() override;

    void Stop();//停止电机

    static void MotorsHandle();//电机处理函数,在中断中调用,处理所有电机

protected:
    PID speedPID, anglePID;//速度环和角度环PID对象
    float reductionRatio;//减速比为1
    bool stopFlag{true};//停止标志位
    MOTOR_CTRL_TYPE_e ctrlType;//控制电机的方式
private:
    static Motor_Object_t *head_;//电机对象链表头指针

};


/*4315电机类------------------------------------------------------------------*/
class Motor_4315 : public Motor, public RS485 {
public:
    int32_t motor4315_angle[8]{};
    float nowAngle{0};
    float targetAngle{0};
    float zeroAngle{0};
    uint8_t RxMessage[15]{};//接收到的电机数据
    void RS485MessageGenerate() override;

    void Handle() override;

    void SetTargetAngle(float _targetAngle);

    Motor_4315(uint32_t _id, MOTOR_INIT_t *_init);

    ~Motor_4315();

private:
    float realAngle{0};
    float lastAngle{0};
    void AngleCalc();
    uint16_t CRC16Calc(uint8_t *data, uint16_t length);

};

/*4*4010电机类------------------------------------------------------------------*/
/**
 * @class FOUR_Motor_4010
 * @brief 4*4010电机类
 * @example FOUR_Motor_4010 motor4010(&commu_init1,&commu_init2,&commu_init3,&commu_init4,&motor_init1,&motor_init2,&motor_init3,&motor_init4);
 * @note 使用该类前需要使用上位机打开电机的多电机模式
 */
class FOUR_Motor_4010 : public Motor, public CAN {
public:
    uint8_t RxMessage[4][8]{};//接收到的电机数据
    int16_t motor4010_intensity[4]{};//电机转矩

    void CANMessageGenerate() override;//CAN报文生成函数,CAN类纯虚函数的重写

    void Handle() override;//电机处理函数,由MotorsHandle统一调用

    void SetTargetSpeed(const float *_targetSpeed);//设置目标速度

    FOUR_Motor_4010(COMMU_INIT_t *commu_init1, COMMU_INIT_t *commu_init2,
                    COMMU_INIT_t *commu_init3, COMMU_INIT_t *commu_init4, MOTOR_INIT_t *motor_init1,
                    MOTOR_INIT_t *motor_init2,MOTOR_INIT_t *motor_init3,MOTOR_INIT_t *motor_init4);//电机初始化函数

    ~FOUR_Motor_4010();

private:
    uint32_t canIDs[4]{};//电机ID
    PID speedPIDs[4];//速度环PID对象,若控位置还新建需一个角度环PID对象
    MOTOR_FEEDBACK_t feedback[4]{};//电机反馈数据
    float targetSpeed[4]{};//目标速度
    MOTOR_STATE_t state[4]{};//电机状态,包含速度、角度、转矩、温度

    void MotorStateUpdate(uint32_t id);//电机状态更新函数

    int16_t IntensityCalc(uint32_t id);//转矩计算函数
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
