#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "list.h" // 包含链表相关的头文件

#define DISPLAY_LINE_LENTH  48    // 显示信息的行长度

#define BLOCK_SIZE          1024  // 簇大小
#define BLOCK_NUM           64    // 磁盘的簇的总数
#define FAT_START_OFFSET    2     // FAT总是从2开始，存放数组相对物理实际的偏移量
#define EMPTY               0     // 空元素

#define CAPACITY_FILE       20    // 文件目录容量


/// @brief FAT文件分配表
typedef struct FATtable {
    /// @brief 存储磁盘的状态信息
    /// state
    ///     0x000               空闲簇
    ///     0x001               保留簇
    ///     0x002 - 0xFEF       被占用的簇；指向下一个簇
    ///     0xFF0 - 0xFF6       保留值
    ///     0xFF7               坏簇
    ///     0xFF8 - 0xFFF       文件结束簇
    int state[BLOCK_NUM]; 
    /// @brief 空闲簇的数量
    int free_count;       
} FATtable;
struct FATtable fat_table;

/// @brief 空闲簇链表
Node *free_block_list = NULL;

/// @brief 文件信息
typedef struct fileInfo {
    /// @brief 文件名
    char *name;
    /// @brief 文件起始簇地址, 0 表示无文件
    int entry_address;      
    /// @brief 文件长度
    int lenth;
} fileInfo;

/// @brief 文件目录表
typedef struct fileTable {
    /// @brief 文件信息数组
    fileInfo file_infos[CAPACITY_FILE]; 
    /// @brief 文件个数
    int count;                          
} fileTable;

fileTable filetable;

/// @brief 格式化 FAT 磁盘信息
void initialize() {
    // 初始化FAT文件分配表状态为0，表示空闲簇
    for(int i = 0; i < BLOCK_NUM; i ++) {
        fat_table.state[i] = 0x0;
    }
    fat_table.free_count = BLOCK_NUM; // 初始时所有簇都是空闲的
    filetable.count = 0; // 初始时文件个数为0
    for(int i = 0; i < CAPACITY_FILE; i ++) {
        filetable.file_infos[i].entry_address = 0; // 初始时文件目录表的所有entry地址为0，表示无文件
    }
    // 初始化空闲簇链表，将所有簇加入到空闲簇链表中
    for(int i = 0; i < BLOCK_NUM; i ++) {
        insert_at_tail(&free_block_list, i);
    }
}

/// @brief 打印居中显示的字符串
void print_centered(char *str) {
    int length = strlen(str);
    int padding = (DISPLAY_LINE_LENTH - length) / 2;
    printf("%*s%s%*s\n", padding, "", str, padding, "");
}

/// @brief 显示文件目录表、空闲块表和FAT表
void display() {
    for(int i = 0; i < DISPLAY_LINE_LENTH; i ++) printf("="); printf("\n");
    // 打印文件目录表
    printf("%-24s%24s\n", "File", "Entry");
    for(int i = 0; i < DISPLAY_LINE_LENTH; i ++) printf("-"); printf("\n");
    for(int i = 0; i < CAPACITY_FILE; i ++) {
        if(filetable.file_infos[i].entry_address != 0) {
            char hex_str[6] = "0x";
            sprintf(hex_str + 2, "%03X", filetable.file_infos[i].entry_address);
            printf("%-24s%24s\n", filetable.file_infos[i].name, hex_str);
        }
    }
    printf("\n");
    // 打印空闲块链表
    print_centered("Free Block List");
    for(int i = 0; i < DISPLAY_LINE_LENTH; i ++) printf("-"); printf("\n");
    print_list(free_block_list);
    printf("\n");
    // 打印FAT表状态
    print_centered("All FAT State");
    for(int i = 0; i < DISPLAY_LINE_LENTH; i ++) printf("-"); printf("\n");
    for(int i = 0; i < BLOCK_NUM; i ++) {
        printf("0x%03X ",fat_table.state[i]);
        if((i + 1) % 8 == 0) {
            printf("\n");
        }
    }
    for(int i = 0; i < DISPLAY_LINE_LENTH; i ++) printf("="); printf("\n");
}

/// @brief 创建一个文件（简单考虑这里不对文件的数据存放进行模拟）
/// @param file_name 文件名
/// @param len 文件长度
/// @return 创建文件是否成功
bool create_file(char *file_name, int len) {
    // 检查空闲簇数量和文件目录表容量
    if(fat_table.free_count < len || filetable.count >= CAPACITY_FILE) {
        printf("!!! Free blocks are not enough\n");
        return false;
    }
    // 检查文件名是否已存在
    for(int i = 0; i < CAPACITY_FILE; i ++) {
        if(
            filetable.file_infos[i].entry_address != 0 && 
            strcmp(file_name, filetable.file_infos[i].name) == 0
        ) {
            printf("!!! %s is already existed\n", file_name);
            return false;
        }
    }
    
    fat_table.free_count -= len; // 更新空闲簇数量
    fileInfo file_info;
    file_info.name = (char *)malloc(strlen(file_name) + 1); // 为文件名分配内存并复制文件名
    strcpy(file_info.name, file_name);
    file_info.lenth = len;
    file_info.entry_address = free_block_list->data + FAT_START_OFFSET; // 获取文件起始簇地址
    delete_first_element(&free_block_list); // 删除空闲簇链表的头节点，表示该簇被占用

    int addr = file_info.entry_address - FAT_START_OFFSET; // 计算FAT表中的下标
    // 更新FAT表中的状态，将对应文件分配的簇状态设置为文件结束簇（0xFFF）或者指向下一个簇
    for(int i = 1; i <= len; i ++) {
        if(i == len) {
            fat_table.state[addr] = 0xFFF; // 最后一个簇的状态为文件结束簇
        } else {
            fat_table.state[addr] = free_block_list->data + FAT_START_OFFSET; // 非最后一个簇指向下一个簇
            addr = free_block_list->data;
            delete_first_element(&free_block_list);
        }
    }

    // 将文件信息添加到文件目录表中
    for(int i = 0; i < CAPACITY_FILE; i ++) {
        if(filetable.file_infos[i].entry_address == 0) {
            filetable.file_infos[i] = file_info;
            filetable.count ++;
            break;
        }
    }
    display(); // 显示更新后的文件目录表、空闲块链表和FAT表
    return true; // 文件创建成功
}

/// @brief 存储文件的簇号和文件长度
typedef struct FileBlocks {
    /// @brief 存储簇号的数组
    int *blocks; 
    /// @brief 文件长度
    int lenth;   
} FileBlocks;

/// @brief 查找文件的所有簇号
/// @param file_name 文件名
/// @return 文件的所有簇号和文件的长度
FileBlocks find_file(char *file_name) {
    FileBlocks res;
    res.lenth = 0;
    // 在文件目录表中查找文件名对应的文件信息
    for(int i = 0; i < CAPACITY_FILE; i ++) {
        if( 
            filetable.file_infos[i].entry_address               != 0 &&
            strcmp(file_name, filetable.file_infos[i].name)     == 0
        ) {
            res.blocks = (int*)malloc(sizeof(int) * filetable.file_infos[i].lenth); // 分配存储簇号的数组内存
            res.lenth = filetable.file_infos[i].lenth; // 更新文件长度
            int addr = filetable.file_infos[i].entry_address;
            int i = 0;
            // 遍历FAT表，获取文件的所有簇号
            do {
                res.blocks[i ++] = addr; // 将簇号存入数组中
                addr = fat_table.state[addr - FAT_START_OFFSET]; // 获取下一个簇号
            } while(addr >= 0x002 && addr <= 0xFEF); // 循环直到遇到文件结束簇
            break;
        }
    }
    // 输出文件的簇号和长度信息
    if(res.lenth == 0) {
        printf("%s is't existed\n", file_name);
    } else {
        printf("find %s, data in: \n", file_name);
        for(int i = 0; i < res.lenth; i ++) {
            printf("0x%03X ", res.blocks[i]);
            if((i + 1) % 8 == 0) printf("\n");
        }
        printf("\n");
    }
    return res;
}

/// @brief 删除文件
/// @param file_name 要删除的文件名
/// @return 是否成功删除文件
bool delete_file(char *file_name) {
    // 遍历文件目录表，查找要删除的文件名
    for(int i = 0; i < CAPACITY_FILE; i ++) {
        // 当文件名存在且文件entry地址不为0时执行删除操作
        if( 
            filetable.file_infos[i].entry_address               != 0 &&
            strcmp(file_name, filetable.file_infos[i].name)     == 0 
        ) {
            int addr = filetable.file_infos[i].entry_address;
            // 遍历FAT表，释放文件所占用的簇并将簇添加到空闲簇链表中
            do {
                int vaddr = addr - FAT_START_OFFSET;
                insert_at_tail(&free_block_list, vaddr);
                addr = fat_table.state[vaddr];
                fat_table.state[vaddr] = 0x000; // 将文件分配的簇状态设置为空闲
            } while(addr >= 0x002 && addr <= 0xFEF); // 循环直到遇到文件结束簇
            fat_table.free_count += filetable.file_infos[i].lenth; // 更新空闲簇数量
            filetable.file_infos[i].entry_address = 0; // 清空文件entry地址
            printf("%s is deleted\n", file_name); // 输出文件删除成功信息
            free(filetable.file_infos[i].name); // 释放文件名内存
            display(); // 显示更新后的文件目录表、空闲块链表和FAT表
            return true; // 文件删除成功
        }
    }
    printf("%s is't existed\n", file_name); // 输出文件不存在信息
    return false; // 文件不存在，删除失败
}

int main () {
    initialize(); // 初始化文件系统
    do {
        char op[8];
        char file[30];
        int lenth;
        // 提示用户输入操作和文件信息
        // printf("input operation filename (lenth): ");
        scanf("%s", op);
        // 根据用户输入执行相应的操作
        if(strcmp(op, "exit") == 0) {
            return 0; // 退出程序
        } else if(strcmp(op, "CRE") == 0) {
            scanf("%s %d", file, &lenth); // 输入文件名和长度
            create_file(file, lenth); // 创建文件
        } else if(strcmp(op, "FIN")  == 0) {
            scanf("%s", file); // 输入要查找的文件名
            find_file(file); // 查找文件
        } else if(strcmp(op, "DEL")  == 0) {
            scanf("%s", file); // 输入要删除的文件名
            delete_file(file); // 删除文件
        } else {
            printf("!!! Operate Error\n"); // 操作错误提示
        }
    } while(true); // 无限循环，直到用户选择退出
}

/*
示例操作：
CRE a.h 24
CRE a.h 10
FIN a.h
CRE a.c 24
CRE b.h 17
CRE b.h 15
FIN b.h
DEL a.c
CRE 1.txt 6
FIN 1.txt
*/

/*
CRE 7 25
FIN 8
CRE 5 40
DEL 8
CRE 2 1
CRE 7 19
FIN 8
DEL 8
FIN 8
CRE 8 28
*/