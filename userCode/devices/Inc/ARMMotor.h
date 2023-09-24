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


/*步进电机V4类------------------------------------------------------------------*/
class SteppingMotor_v4 : public Motor, public CAN {
public:
    uint8_t RxMessage[8]{0};

    float NowPos = 0;
    float TarPos = 0;

    SteppingMotor_v4(COMMU_INIT_t *commuInit, MOTOR_INIT_t *motorInit);

    ~SteppingMotor_v4();

    void Handle() override;


    void SetTargetPosition(float pos);

private:
    uint8_t Direction{};
    uint16_t Speed{1500};
    uint32_t Pulse{0};

    bool SendFlag = false;
    uint8_t TxMessage[8]{0};
    uint8_t TxMessageDLC{};

    void CANMessageGenerate() override;
};

/*步进电机V5类------------------------------------------------------------------*/
class SteppingMotor_v5 : public Motor, public CAN {
public:
    uint8_t RxMessage[8]{0};

    float NowPos = 0;
    float TarPos = 0;
    SteppingMotor_v5(COMMU_INIT_t *commuInit, MOTOR_INIT_t *motorInit);

    void Handle() override;

    void SetTargetPosition(float tarpos);

    ~SteppingMotor_v5();

private:

    uint8_t Direction{};
    uint16_t Speed{1500};
    uint32_t Pulse{};

    bool SendFlag = false;
    uint8_t TxMessage[8]{0};
    uint8_t TxMessageDLC{};

    int32_t nowPos = 0;

    void CANMessageGenerate() override;

};

#endif //RM_FRAME_C_ARMMOTOR_H
