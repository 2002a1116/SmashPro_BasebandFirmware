/*
 * conf.c
 *
 *  Created on: 2024年11月26日
 *      Author: Reed
 */

 #include "flash.h"
 #include "uart.h"
 #include "conf.h"
 #include "esp_log.h"

 #include <string.h>
 
 factory_configuration_data factory_configuration;
 user_calibration_data user_calibration;
 user_config_data user_config;
 uint32_t joystick_snapback_deadzone_sq[2];
 static factory_configuration_flash_pack* fac;
 #define JOYSTICK_RANGE_FACTOR_LEFT (0.6f)
 #define JOYSTICK_RANGE_FACTOR_RIGHT (0.6f)
 uart_packet pkg;
 int32_t i32_min(int32_t a,int32_t b){return a<b?a:b;}
 void conf_init()
 {
    pkg.typ=UART_PKG_CH32_FLASH_READ;
    pkg.id=0xF;
    for(int i=0;i<sizeof(user_config);i+=8){
        pkg.load[0]=i;
        send_uart_pkt(&pkg);
        send_uart_pkt(&pkg);
    }
    pkg.id=0x6;
    for(int i=0;i<sizeof(factory_configuration);i+=8){
        ESP_LOGE(__func__,"init 0x6 offset:%d",i);
        pkg.load[0]=i;
        send_uart_pkt(&pkg);
        send_uart_pkt(&pkg);
    }
    pkg.id=0x8;
    for(int i=0;i<sizeof(user_calibration);i+=8){
        pkg.load[0]=i;
        send_uart_pkt(&pkg);
        send_uart_pkt(&pkg);
    }
 }
 void uart_config_cb(uart_packet* p){
    if(!p)return;
    if(p->typ!=UART_PKG_CH32_FLASH_READ&&p->typ!=UART_PKG_CH32_FLASH_WRITE)return;
    if(p->typ==UART_PKG_CH32_FLASH_READ){
        ESP_LOGW(__func__,"recv %d offset:%d",p->id,p->load[0]);
        switch(p->id){
            case 0xF://user config
                memcpy(((uint8_t*)&user_config)+p->load[0],p->load+1,i32_min(8,sizeof(user_config)-p->load[0]));
                break;
            case 0x6:
                memcpy(((uint8_t*)&factory_configuration)+p->load[0],p->load+1,i32_min(8,sizeof(factory_configuration)-p->load[0]));
                break;
            case 0x8:
                memcpy(((uint8_t*)&user_calibration)+p->load[0],p->load+1,i32_min(8,sizeof(user_calibration)-p->load[0]));
                break;
            default:
                break;
        }
    }else{
        switch(p->id){
            default:
                break;
        }
    }
 }
 void conf_read(uint32_t addr,uint8_t* buf,uint8_t size){
     switch(addr & 0xffff00)
     {
     case 0x5000:
         break;
     case 0x6000:
         memcpy(buf,((uint8_t*)&factory_configuration)+(addr&0xff),size);
         break;
     case 0x8000:
         memcpy(buf,((uint8_t*)&user_calibration)+(addr&0xff),size);
         break;
     default:
         break;
     }
 }
 void uart_conf_write(uint32_t addr,uint8_t* ptr,uint8_t size)
 {
    pkg.typ=UART_PKG_CH32_FLASH_WRITE;
    pkg.id=addr>>24;
    addr&=0xff;
    for(int i=0;i<size;i+=8){
        pkg.load[0]=i+addr;
        memcpy(pkg.load+1,ptr,8);
        ptr+=8;
        send_uart_pkt(&pkg);
    }
 }
 uint8_t conf_write(uint32_t addr,uint8_t* buf,uint8_t size){
     uint8_t flash_res=0;
     switch(addr & 0xffff00)
     {
     case 0x5000:
         break;
     case 0x6000:
         memcpy(((uint8_t*)&factory_configuration)+(addr&0xff),(uint8_t*)buf,size);
         uart_conf_write(addr,((uint8_t*)&factory_configuration)+(addr&0xff),size);
         break;
     case 0x8000:
         memcpy(((uint8_t*)&user_calibration)+(addr&0xff),(uint8_t*)buf,size);
         uart_conf_write(addr,((uint8_t*)&factory_configuration)+(addr&0xff),size);
         break;
     default:
         break;
     }
     ////printf("conf write res: %d\r\n",flash_res);
     return flash_res;
 }
 void unpack_fac_conf()
 {
    
 }
 void fac_conf_read(){

 }
 uint8_t fac_conf_write(){
    return 0;
 }
 void conf_flush(){

 }
 