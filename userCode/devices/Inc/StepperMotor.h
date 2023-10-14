//
// Created by 25396 on 2023/9/7.
//

#ifndef RM_FRAME_C_STEPPERMOTOR_H
#define RM_FRAME_C_STEPPERMOTOR_H

#include "Device.h"
#include "Motor.h"
#include "ManiControl.h"

class StepperMotor : public Motor {
public:
    uint8_t DIR{};
    uint16_t STEP{};
    bool posflag{true};

    explicit StepperMotor(MOTOR_INIT_t *_init);

    ~StepperMotor();

    void Handle() override;

    void Grab(bool _posflag);
};


#endif //RM_FRAME_C_STEPPERMOTOR_H
