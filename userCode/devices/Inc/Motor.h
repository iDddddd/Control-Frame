//
// Created by LEGION on 2021/10/4.
//

#ifndef RM_FRAME_C_MOTOR_H
#define RM_FRAME_C_MOTOR_H

#include "Device.h"
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
    float reductionRatio;//减速比
    bool stopFlag{true};//停止标志位
    MOTOR_CTRL_TYPE_e ctrlType;//控制电机的方式
private:
    static Motor_Object_t *head_;//电机对象链表头指针

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
