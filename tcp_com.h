#pragma once
#define STR_FIND_INVALID (-1)
typedef struct _tcp_cmd_handler_discriber_struct{
    char* cmd;
    int cmd_len;
    void (*handler)(char*,int);
    struct _tcp_cmd_handler_discriber_struct* nxt;
}tcp_cmd_handler_discriber;
void tcp_cmd_register_handler(char* cmd,int cmd_len,void (*handler)(char*,int));
uint32_t str_find_char(char* ptr,int len,char c);
void tcp_cmd_dispatcher(char* cmd,int len);