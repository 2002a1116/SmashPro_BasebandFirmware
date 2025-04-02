#pragma once
void wifi_init(void (*on_recv)(char*,int,void*,int),void* context,int context_len);
static void do_send(const int,void*,int);
void tcp_send(void* buf,int len);