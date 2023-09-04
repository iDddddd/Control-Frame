//
// Created by mac on 2022/12/14.
//

#ifndef RM_FRAME_C_ARMMOTOR_H
#define RM_FRAME_C_ARMMOTOR_H

#include "Device.h"
#include "can.h"
#include "ManiControl.h"
#include "Motor.h"
#include "CommuType.h"

#define DOWN  0
#define MID  1
#define UP  2


#define RED  0
#define BLUE 1
#define GREEN 2


/*步进电机类------------------------------------------------------------------*/
class SteppingMotor : public Motor, public CAN {
public:

    SteppingMotor(COMMU_INIT_t *commuInit, MOTOR_INIT_t *motorInit);

    uint8_t RxMessage[6]{0};
    uint8_t TxMessage[8]{0};
    bool SendFlag = false;
    float NowPos = 0;
    float TarPos = 0;
    uint8_t Direction{};
    float Position{};
    uint16_t Speed{};
    uint32_t Pulse{};

    ~SteppingMotor();

    void CANMessageGenerate() override;

    void Handle() override;
    void MoveTo();
    void SetTargetPosition(float pos);

private:
    float targetPosition{}; //单位mm

};

#endif //RM_FRAME_C_ARMMOTOR_H
