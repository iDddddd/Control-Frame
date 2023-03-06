//
// Created by mac on 2022/12/14.
//

#ifndef RM_FRAME_C_OTHERMOTOR_H
#define RM_FRAME_C_OTHERMOTOR_H

#include "Device.h"
#include "can.h"
#include "CatchControl.h"
#include "Motor.h"
#include "CommuType.h"

typedef enum {
    DOWN = 0,
    UP
}MOTORZ_POS_t;
/*4010电机类------------------------------------------------------------------*/
class Motor_4010 : public Motor, public CAN {
public:
    uint8_t RxMessage[8]{};
    int16_t motor4010_intensity[8]{};

    void CANMessageGenerate() override;

    void Handle() override;

    void SetTargetAngle(float _targetAngle);

    Motor_4010(COMMU_INIT_t *commuInit, MOTOR_INIT_t *motorInit);

    ~Motor_4010();

private:
    uint8_t id;
    C6x0Rx_t feedback{};
    float targetAngle{};
    MOTOR_STATE_t state{};
    float realAngle{0};
    float thisAngle;
    float lastRead;
    static int16_t Intensity;

    void MotorStateUpdate();

    int16_t IntensityCalc();
};

/*4310电机类------------------------------------------------------------------*/
class Motor_4310 : public Motor, public CAN {
public:
    uint8_t* Motor4310_Angle;
    uint32_t sendSpeed{};

    static void Init();

    void CANMessageGenerate() override;

    void Handle() override;

    void SetTargetAngle(float _targetAngle);

    void SetTargetSpeed(float _targetSpeed);

    Motor_4310(COMMU_INIT_t *commuInit, MOTOR_INIT_t *motorInit);

    ~Motor_4310();

private:
    float targetAngle{};
    float targetSpeed{};

};

/*Emm42电机类------------------------------------------------------------------*/
class Emm42Motor : public Motor, public CAN {
public:
    Emm42Motor(COMMU_INIT_t *commuInit, MOTOR_INIT_t *motorInit);
    MOTORZ_POS_t NowPos = DOWN;
    MOTORZ_POS_t TarPos = DOWN;
    uint8_t Emm42Motor_Dir{};
    uint32_t Emm42Motor_Pos{};
    ~Emm42Motor();

    void CANMessageGenerate() override;

    void Handle() override;

    void SetTargetPosition(MOTORZ_POS_t pos);

private:
    float targetPosition{}; //单位mm

};

#endif //RM_FRAME_C_OTHERMOTOR_H
