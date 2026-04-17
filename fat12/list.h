typedef struct Node {
    int data;               // 节点数据
    struct Node* next;      // 指向下一个节点的指针
} Node;

/// @brief 创建一个新的链表节点
/// @param data 节点的数据
/// @return Node* 返回指向新节点的指针
Node* create_node(int data);

/// @brief 将新节点插入到链表尾部
/// @param head 指向链表头指针的指针
/// @param data 新节点的数据
void insert_at_tail(Node** head, int data);

/// @brief 读取链表的第一个节点的数据
/// @param head 链表的头指针
/// @return int 返回第一个节点的数据
int read_first_element(Node* head);

/// @brief 删除链表的第一个节点
/// @param head 指向链表头指针的指针
void delete_first_element(Node** head);

/// @brief 打印链表的所有节点数据
/// @param head 链表的头指针
void print_list(Node* head);

/// @brief 释放链表的所有节点内存
/// @param head 链表的头指针
void free_list(Node* head);