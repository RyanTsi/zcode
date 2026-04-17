#include <stdio.h>
#include <stdlib.h>
#include "list.h"

// 创建新节点
Node* create_node(int data) {
    Node* new_node = (Node*)malloc(sizeof(Node));
    if (new_node == NULL) {
        printf("Memory allocation failed\n");
        exit(1);
    }
    new_node->data = data;
    new_node->next = NULL;
    return new_node;
}

// 在链表尾部插入新节点
void insert_at_tail(Node** head, int data) {
    Node* new_node = create_node(data);
    if (*head == NULL) {
        *head = new_node;
        return;
    }
    Node* temp = *head;
    while (temp->next != NULL) {
        temp = temp->next;
    }
    temp->next = new_node;
}

// 读取第一个节点的值
int read_first_element(Node* head) {
    if (head == NULL) {
        printf("List is empty\n");
        return -1; // 返回一个错误值，表示链表为空
    }
    return head->data;
}

// 删除第一个节点
void delete_first_element(Node** head) {
    if (*head == NULL) {
        printf("List is empty, nothing to delete\n");
        return;
    }
    Node* temp = *head;
    *head = (*head)->next;
    free(temp);
}

// 打印链表中的所有节点
void print_list(Node* head) {
    int i = 0;
    Node* temp = head;
    while (temp != NULL) {
        printf("0x%03X ", temp->data);
        if((++i) % 8 == 0) {
            printf("\n");
        }
        temp = temp->next;
    }
}

// 释放链表中的所有节点
void free_list(Node* head) {
    Node* temp;
    while (head != NULL) {
        temp = head;
        head = head->next;
        free(temp);
    }
}