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

    void MotorStateUpdate();

    int16_t IntensityCalc();
};
/*4310电机类------------------------------------------------------------------*/
class Motor_4310 : public Motor, public CAN {
public:
    static uint8_t InitMessage[2];
    int16_t motor4310_intensity[8];
    static void Init();
    void CANMessageGenerate() override;

    void Handle() override;

    void SetTargetAngle(float _targetAngle);
    void SetTargetSpeed(float _targetSpeed);



    Motor_4310(COMMU_INIT_t *commuInit, MOTOR_INIT_t *motorInit);

    ~Motor_4310();

private:

    float targetAngle;
    float targetSpeed;

};
#endif //RM_FRAME_C_OTHERMOTOR_H
