#ifndef RM_FRAME_C_COMMUTYPE_H
#define RM_FRAME_C_COMMUTYPE_H

#include <queue>
#include <map>

#include "Device.h"
#include "Map.h"

#define MAX_MESSAGE_COUNT 20
#define RX_SIZE 20
#define MOTOR_RX_SIZE 15u

constexpr uint32_t CAN2_MASK = 1u << 30;
#define CAN1_ID(_id) (_id)
#define CAN2_ID(_id) (_id + CAN2_MASK)

#define RS485_ID(_id) (_id)

/*结构体定义--------------------------------------------------------------*/

typedef struct {
    uint32_t ID;
    uint8_t DLC;
    uint8_t message[8];
}DATA_t;

/*class CANQueue {
public:
    bool empty() const {
        if(top == rear) return true;
        else return false;
    }

    DATA_t& front() {
        return data[top];
    }

    DATA_t pop() {
        DATA_t retval = data[top];
        top = (top + 1) % MAX_MESSAGE_COUNT;
        return retval;
    }

    void emplace(DATA_t&& input) {
        if ((rear + 1) % MAX_MESSAGE_COUNT != top) {
            data[rear] = input;
            rear = (rear + 1) % MAX_MESSAGE_COUNT;
        }
    }

private:
    DATA_t data[MAX_MESSAGE_COUNT] = {0};
    size_t top = 0, rear = 0;
};*/

/*类型定义----------------------------------------------------------------*/
/*CAN类------------------------------------------------------------------*/
/**
 * @class CAN类
 */
class CAN {
public:
    uint32_t ID;//CAN ID
    //static CANQueue canQueue;//CAN发送队列
    static std::queue<DATA_t> canQueue;

    static void CANInit();//CAN初始化函数,需在main函数中调用，无需过多关注

    explicit CAN(uint32_t id);//默认构造函数，用于设置canID和canType

    ~CAN();

    static void CANPackageSend();//can消息包发送任务

    static void Rx_Handle(CAN_HandleTypeDef *hcan);

    virtual void CANMessageGenerate() = 0;//can消息包生成函数,需在每个电机类中实现

protected:
//    static std::map<uint32_t, uint8_t *> dict_CAN;//canID与can回传消息的字典
    static MyMap<uint32_t, uint8_t *> dict_CAN;
    void ID_Bind_Rx(uint8_t *RxMessage) const;//将canID与can回传消息绑定
};

/*RS485类------------------------------------------------------------------*/
class RS485 {
public:
    uint32_t rs485_ID;
    static uint8_t rsmessage[4][11];

    static uint8_t rs485_rx_buff[2][RX_SIZE];

    explicit RS485(uint32_t _id);

    ~RS485();

    static void RS485Init();
    static void RS485PackageSend();
    static void Rx_Handle();
    virtual void RS485MessageGenerate() = 0;

protected:
    static MyMap<uint32_t, uint8_t *> dict_RS485; //485ID与485回传消息的字典
    void ID_Bind_Rx(uint8_t *RxMessage) const; //将485ID与485回传消息绑定

};

#ifdef __cplusplus
extern "C" {
#endif
extern void USART1_IRQHandler(void);
#ifdef __cplusplus
}
#endif

#endif //RM_FRAME_C_COMMUTYPE_H
