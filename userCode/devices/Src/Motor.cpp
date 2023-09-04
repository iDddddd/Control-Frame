//
// Created by LEGION on 2021/10/4.
//

#include "Motor.h"


Motor_Object_t *Motor::head_;

/*Motor类----------------------------------------------------------------*/
/**
 * @brief Motor类的构造函数
 * @param _init 类的初始化结构体指针
 */
Motor::Motor(MOTOR_INIT_t *_init, Motor *motor) {
    deviceType = MOTOR;//设备类型为电机

    if (_init->speedPIDp) speedPID.PIDInfo = *_init->speedPIDp;//如果速度环pid参数结构体指针不为空，则将参数结构体赋值给类的参数结构体
    if (_init->anglePIDp) anglePID.PIDInfo = *_init->anglePIDp;//如果角度环pid参数结构体指针不为空，则将参数结构体赋值给类的参数结构体
    ctrlType = _init->ctrlType;//控制电机的方式
    reductionRatio = _init->reductionRatio;//减速比
    //将电机对象加入电机链表，实现后续中断处理
    auto *new_object = new Motor_Object_t();
    new_object->motor_object = motor;
    new_object->next = head_;
    head_ = new_object;

}

/**
* @brief Motor类的析构函数
*/
Motor::~Motor() = default;

void Motor::ErrorHandle() {}

void Motor::MotorsHandle() {
    Motor_Object_t *current = head_;
    while (current) {
        current->motor_object->Handle();//调用每个电机的中断处理函数
        current = current->next;
    }

}

void Motor::Stop() {
    stopFlag = true;

}


