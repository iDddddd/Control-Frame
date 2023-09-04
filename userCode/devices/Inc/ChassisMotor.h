//
// Created by 25396 on 2023/9/3.
//

#ifndef RM_FRAME_C_CHASSISMOTOR_H
#define RM_FRAME_C_CHASSISMOTOR_H

#include "Device.h"
#include "CommuType.h"
#include "Motor.h"
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


/*4010电机类------------------------------------------------------------------*/
class Motor_4010 : public Motor, public CAN {
public:
    uint8_t RxMessage[8]{};
    int16_t motor4010_intensity{};
    int32_t txPos{};
    uint16_t txSpeed{300};
    float targetSpeed{};
    MOTOR_FEEDBACK_t feedback{};
    MOTOR_STATE_t state{};
    float vx, vy;
    float target_vx, target_vy;

    void CANMessageGenerate() override;

    void Handle() override;

    void SetTargetAngle(float _targetAngle);

    void SetTargetSpeed(float _targetSpeed);

    Motor_4010(COMMU_INIT_t *commuInit, MOTOR_INIT_t *motorInit);

    ~Motor_4010();

private:
    uint8_t id;
    //位置模式
    float targetAngle{};

    float realAngle{0};
    float thisAngle{};
    float lastRead{};

    void MotorStateUpdate();

    int16_t IntensityCalc();

    void StopMoving();
};


#endif //RM_FRAME_C_CHASSISMOTOR_H
