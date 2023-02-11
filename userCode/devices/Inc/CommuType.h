//
// Created by 25396 on 2023/2/11.
//

#ifndef RM_FRAME_C_COMMUTYPE_H
#define RM_FRAME_C_COMMUTYPE_H

#include "Device.h"
#include <map>
/*结构体定义--------------------------------------------------------------*/
typedef enum {
    DIRECT = 0,
    SPEED_Single,
    POSITION_Double
} MOTOR_CTRL_TYPE_e;

typedef struct {
    uint16_t _id;//canID
    MOTOR_CTRL_TYPE_e ctrlType;
} COMMU_INIT_t;

/*类型定义----------------------------------------------------------------*/
/*CAN类------------------------------------------------------------------*/
class CAN {
public:
    uint16_t can_ID;
    static uint8_t canmessage[8];

    static void CANInit();

    CAN();

    explicit CAN(COMMU_INIT_t *_init);

    ~CAN();

    static void CANPackageSend();

    static void Rx_Handle(CAN_HandleTypeDef *hcan);

    virtual void CANMessageGenerate() = 0;

protected:

    MOTOR_CTRL_TYPE_e ctrlType;
    static std::map<uint16_t, uint8_t *> dict;

    void ID_Bind_Rx(uint8_t *RxMessage);

    void FOURID_Bind_Rx(uint16_t *canID, uint8_t (*RxMessage)[8]);
};

/*RS485类------------------------------------------------------------------*/
class RS485 {
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


#endif //RM_FRAME_C_COMMUTYPE_H
