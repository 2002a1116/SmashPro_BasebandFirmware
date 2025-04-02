#include <string.h>
#include "stdio.h"
#include "esp_log.h"

#include "tcp_com.h"

tcp_cmd_handler_discriber* tcp_cmd_handler_list=NULL;
uint32_t str_find_char(char* ptr,int len,char c){
    if(!ptr)return STR_FIND_INVALID;
    for(int i=0;i<len;++i){
        if(ptr[i]==c)
            return i;
    }
    return STR_FIND_INVALID;
}
void tcp_cmd_register_handler(char* cmd,int cmd_len,void (*handler)(char*,int)){
    tcp_cmd_handler_discriber* ptr=malloc(sizeof(tcp_cmd_handler_discriber));
    //ptr->cmd=cmd;
    //shallow copy
    ptr->cmd=malloc(cmd_len);
    memcpy(ptr->cmd,cmd,cmd_len);
    //deep copy cmd
    ptr->cmd_len=cmd_len;
    ptr->handler=handler;
    ptr->nxt=tcp_cmd_handler_list;
    tcp_cmd_handler_list=ptr;
}

void tcp_cmd_dispatcher(char* cmd,int len){
    if(len<2)return;
    __uint8_t tag=0;
    tcp_cmd_handler_discriber* ptr=tcp_cmd_handler_list;
    int cmd_len=str_find_char(cmd,len,' ');
    if(cmd_len==STR_FIND_INVALID)
        cmd_len=len-1;
    while(ptr){
        if(cmd_len==ptr->cmd_len&&!strncmp(cmd,ptr->cmd,cmd_len))//find
        {
            if(ptr->handler)
                ptr->handler(cmd+cmd_len+1,len-cmd_len-1);
            tag=1;
            break;
        }
        ptr=ptr->nxt;
    }
    if(!tag){
        ESP_LOGW("TCP CMD","dispatch fail %s",cmd);
    }
}