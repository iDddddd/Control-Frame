//
// Created by 25396 on 2023/4/6.
//

#include "StateMachine.h"
//全局变量声明
FunctionList StateMachine::function_lists = FunctionList();

// 函数指针链表
void FunctionList::add(void (*f)()) {
    //检查链表中是否已有该函数
    FunctionNode *curr = head;
    while (curr) {
        if (curr->func == f) {
            return;
        }
        curr = curr->next;
    }
    //链表中没有该函数，添加
    if (!head) {
        head = new FunctionNode(f);
    } else {
        curr = head;
        while (curr->next) {
            curr = curr->next;
        }
        curr->next = new FunctionNode(f);
    }

}

void FunctionList::remove(void (*f)()) {
    FunctionNode *curr = head;
    FunctionNode *prev = nullptr;
    while (curr) {
        if (curr->func == f) {
            if (prev) {
                prev->next = curr->next;
            } else {
                head = curr->next;
            }
            delete curr;
            break;
        }
        prev = curr;
        curr = curr->next;
    }
}

void FunctionList::call_all() {
    FunctionNode *curr = head;
    while (curr) {
        curr->func();
        curr = curr->next;
    }
}


void StateMachine::stateHandle() {
    function_lists.call_all();  // 触发当前状态对应的所有函数
}

void StateMachine::add_function_to_state(void (*f)()) {
    function_lists.add(f);
}

void StateMachine::remove_function_from_state(void (*f)()) {
    function_lists.remove(f);
}

StateMachine::StateMachine() {
    function_lists = FunctionList();// 初始化函数指针链表
}

