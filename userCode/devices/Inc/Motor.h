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
typedef struct {
    int16_t angle;
    int16_t speed;
    int16_t moment;
    int8_t temp;
} C6x0Rx_t;

typedef struct {
    PID_Regulator_t *speedPIDp;//速度环pid参数结构体指针
    PID_Regulator_t *anglePIDp;//角度环pid参数结构体指针
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
};

/*类型定义----------------------------------------------------------------*/

/*Motor类----------------------------------------------------------------*/
class Motor : private Device {
public:
    Motor(MOTOR_INIT_t *_init, Motor *motor);

    ~Motor();

    void ErrorHandle() override;

    void Stop();

    static void MotorsHandle();

protected:
    PID speedPID, anglePID;
    float reductionRatio;
    bool stopFlag{true};
private:
    static Motor_Object_t *head_;

};


/*4315电机类------------------------------------------------------------------*/
class Motor_4315 : public Motor, public RS485 {
public:
    int16_t motor4315_intensity[8]{};
    float targetAngle = 0;

    void RS485MessageGenerate() override;

    void Handle() override;

    void SetTargetAngle(float _targetAngle);

    Motor_4315(uint32_t _id, MOTOR_INIT_t *_init);

    ~Motor_4315();

private:
    uint16_t CRC16Calc(uint8_t *data, uint16_t length);

};

/*4*4010电机类------------------------------------------------------------------*/
class FOUR_Motor_4010 : public Motor, public CAN {
public:
    uint8_t RxMessage[4][8]{};
    int16_t motor4010_intensity[4]{};

    void CANMessageGenerate() override;

    void Handle() override;

    void SetTargetSpeed(const float *_targetSpeed);

    FOUR_Motor_4010(COMMU_INIT_t *commu_init1, COMMU_INIT_t *commu_init2,
                    COMMU_INIT_t *commu_init3, COMMU_INIT_t *commu_init4, MOTOR_INIT_t *motor_init1,
                    MOTOR_INIT_t *motor_init2);

    ~FOUR_Motor_4010();

private:
    uint32_t canIDs[4]{};
    PID speedPIDs[4];
    C6x0Rx_t feedback[4]{};
    float targetSpeed[4]{};
    MOTOR_STATE_t state[4]{};

    void MotorStateUpdate(uint32_t id);

    int16_t IntensityCalc(uint32_t id);
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
