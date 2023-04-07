//
// Created by 25396 on 2023/4/6.
//

#ifndef RM_FRAME_C_STATEMACHINE_H
#define RM_FRAME_C_STATEMACHINE_H

// 函数指针链表节点
class FunctionNode {
public:
    explicit FunctionNode(void (*f)()) : func(f), next(nullptr) {}
    void (*func)();
    FunctionNode* next;
};
// 函数指针链表
class FunctionList {
public:
    FunctionList() : head(nullptr) {}
    void add(void (*f)());
    void remove(void (*f)());
    void call_all() ;
private:
    FunctionNode* head;
};
// 有限状态机
class StateMachine {
public:
    StateMachine();
    static void stateHandle() ;
    static void add_function_to_state(void (*f)());
    static void remove_function_from_state(void (*f)());
private:
    static FunctionList function_lists;
};


#endif //RM_FRAME_C_STATEMACHINE_H
