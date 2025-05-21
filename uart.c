#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "uart.h"
#include "bt_hid_manager.h"
#include "flash.h"
#include "pwr.h"
#include "conf.h"
static DRAM_ATTR peripheral_data input_data;
static DRAM_ATTR uint8_t uart_recv_buf[1024];
static TaskHandle_t uart_recv_task_handle;
static SemaphoreHandle_t uart_send_mutex;
uint8_t bt_status;
void uart_init()
{
    uart_send_mutex=xSemaphoreCreateMutex();
    uart_config_t uart_config = {
        .baud_rate = INTERNAL_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_intr_config_t uart_intr = {
        .rx_timeout_thresh=10,
        .rxfifo_full_thresh=1,
        .txfifo_empty_intr_thresh=10,
    };
    ESP_ERROR_CHECK(uart_driver_install(INTERNAL_UART_PORT_NUM, INTERNAL_UART_BUF_SIZE * 2, INTERNAL_UART_BUF_SIZE / 2, 0, NULL, 
                    INTERNAL_UART_INTR_ALLOC_FLAGS));
    ESP_ERROR_CHECK(uart_param_config(INTERNAL_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_intr_config(INTERNAL_UART_PORT_NUM,&uart_intr));
    if(INTERNAL_UART_PORT_NUM==2)
        ESP_ERROR_CHECK(uart_set_pin(
        INTERNAL_UART_PORT_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    else if(INTERNAL_UART_PORT_NUM==1)
        ESP_ERROR_CHECK(uart_set_pin(
        INTERNAL_UART_PORT_NUM, INTERNAL_UART_TXD_GPIO_NUM, INTERNAL_UART_RXD_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    xTaskCreatePinnedToCore(uart_recv_task,"uart_recv_task",4096,NULL,3,&uart_recv_task_handle,0);
}
void IRAM_ATTR uart_get_peripheral_data(peripheral_data* data)
{
    if(!data)return;
    *data=input_data;
}
static uart_packet reply;
void IRAM_ATTR uart_pkt_handler(puart_packet pkg)
{
    //ESP_LOGI("uart_pkt_handler"," typ:%d id:%d\r\n",pkg->typ,pkg->id);
    switch(pkg->typ){
        case UART_PKG_INPUT_DATA:
            //ESP_LOGI("INPUT DATA","TYP %d",pkg->id);
            switch(pkg->id){
                case 1:
                    memcpy(&input_data,pkg->load,9);
                    break;
                default:
                    break;
            }
            break;
        case UART_PKG_CONNECT_CONTROL:
            switch(pkg->id)
            {
                case 0:
                    if(pkg->load[0]){
                        //bt_status|=0x1;
                        //set_bt_status(0);
                        bt_connect_async_config(0,0);
                    }
                    else{
                        //bt_status&=0xfe;
                        //set_bt_status(1);
                        bt_connect_async_config(1,0);
                    }
                    break;
                case 1:
                    reply.typ=UART_PKG_CONNECT_CONTROL;
                    reply.id=0;
                    memcpy(reply.load,bt_addr,6);
                    send_uart_pkt(&reply);
                    break;
                case 2:
                    reply.typ=UART_PKG_CONNECT_CONTROL;
                    reply.id=1;
                    memcpy(reply.load,bt_hid_get_ltk(),9);
                    send_uart_pkt(&reply);
                    memset(reply.data,0,11);
                    reply.typ=UART_PKG_CONNECT_CONTROL;
                    reply.id=2;
                    memcpy(reply.load,bt_hid_get_ltk()+9,7);
                    send_uart_pkt(&reply);
                    break;
                case 3:
                    memcpy(con_addr,pkg->load,6);
                    nvs_set("con_addr",con_addr,6);
                    reply.id=0xE;
                    send_uart_pkt(&reply);
                    break;
                case 5:
                    bt_connect_async_config(0,1);
                    //set_connectable();
                    break;
                default:
                    break;
            }
            break;
        case UART_PKG_TEST_ECHO:
            ESP_LOGI("UART_ECHO:","%c%c%c",pkg->load[0],pkg->load[1],pkg->load[2]);
            break;
        case UART_PKG_PWR_CONTROL:
            switch(pkg->id)
            {
                case 0:
                    set_nosleep_bit(NOSLEEP_FORCED,pkg->load[0]);
                    reply.typ=UART_PKG_PWR_CONTROL;
                    reply.id=0;
                    reply.load[0]=pkg->load[0];
                    send_uart_pkt(&reply);
                    break;
                case 1:
                    //uart wkup;
                    break;
            }
            break;
        case UART_PKG_CH32_FLASH_WRITE:
            /*fall through*/
        case UART_PKG_CH32_FLASH_READ:
            uart_config_cb(pkg);
        /*case UART_PKG_LTK:
            reply.typ=UART_PKG_LTK;
            for(int i=0;i*3<15;++i)
            {
                reply.id=i;
                memcpy(reply.arr,bt_hid_get_ltk()+i*3,3);
                send_uart_pkt(&reply);
            }
            reply.id=5;
            memset(reply.data,0,4);
            reply.arr[0]=*(bt_hid_get_ltk()+15);
            send_uart_pkt(&reply);
            break;
            */
        default:
            break;
    }
}
void IRAM_ATTR uart_recv_task()
{
    //uint8_t *data = (uint8_t *) malloc(INTERNAL_UART_BUF_SIZE);
    puart_packet pkt=NULL;
    int len=0,ptr=0,res=0;
    while (1) {
        // Read data from the UART
        vTaskDelay(1/portTICK_PERIOD_MS);
        //taskYIELD();
        res = uart_read_bytes(INTERNAL_UART_PORT_NUM, uart_recv_buf+len, INTERNAL_UART_BUF_SIZE-len, 0);
        //res = uart_read_bytes(INTERNAL_UART_PORT_NUM, uart_recv_buf+len, INTERNAL_UART_BUF_SIZE-len, INTERNAL_UART_GAP / portTICK_PERIOD_MS);
        if(res<0){
            ESP_LOGE(__func__,"res:%d",res);
        }
        //ESP_LOGI(__func__,"");
        if(res==0)continue;
        len+=res;
        //ESP_LOGI("uart_recv","res %d \t len %d",res,len);
        ptr=-1;
        res=0;
        for(int i=0;i<len;++i)
        {
            if(INTERNAL_UART_PKG_HEADER_MASK&uart_recv_buf[i]){
                res=1;
                //printf("first header at %d %d %d\r\n",i,len,uart_recv_buf[i]);
            }
            else if(res){
                ++res;
                while(res>=INTERNAL_UART_PKG_SIZE){
                    pkt=(puart_packet)(uart_recv_buf+i-INTERNAL_UART_PKG_SIZE+1);
                    ptr=i;
                    res=0;
                    if(check_uart_pkt(pkt))
                    {
                        ESP_LOGI("ucf","0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x",*(((uint8_t*)pkt)+0),*(((uint8_t*)pkt)+1),
                        *(((uint8_t*)pkt)+2),*(((uint8_t*)pkt)+3),*(((uint8_t*)pkt)+4),*(((uint8_t*)pkt)+5));
                        //ESP_LOGW("","uart check fail");
                        break;
                    }
                    decode_uart_pkt(pkt);
                    uart_pkt_handler(pkt);
                    break;
                }
            }
            else{
                res=0;
                ptr=i;
            }
        }
        memmove(uart_recv_buf,uart_recv_buf+ptr+1,len=len-ptr-1);
    }
}
void uart_send(puart_packet pkg)
{
    //static portMUX_TYPE uart_spin_lock = portMUX_INITIALIZER_UNLOCKED;
    if(pkg){
        //taskENTER_CRITICAL_FROM_ISR(&uart_spin_lock);
        xSemaphoreTake(uart_send_mutex,portMAX_DELAY);
        //ESP_LOGI(__func__,"uart send typ:%d\tid:%d",pkg->typ,pkg->id);
        uart_write_bytes(INTERNAL_UART_PORT_NUM, (const char *) pkg, INTERNAL_UART_PKG_SIZE);
        xSemaphoreGive(uart_send_mutex);
        //taskEXIT_CRITICAL_FROM_ISR(&uart_spin_lock);
    }
}

void encode_uart_pkt(uart_packet* pkt){
    if(!pkt)return;
    pkt->hb1=0;
    pkt->hb2=0;
    for(int i=0;i<7;++i){
        pkt->hb1|=(pkt->load[i]&INTERNAL_UART_PKG_HEADER_MASK)>>(7^i);//equal high_bit>>7<<i
    }
    pkt->hb2|=(pkt->load[7]&INTERNAL_UART_PKG_HEADER_MASK)>>(7);
    pkt->hb2|=(pkt->load[8]&INTERNAL_UART_PKG_HEADER_MASK)>>(6);
    for(int i=0;i<9;++i)
        pkt->load[i]&=~INTERNAL_UART_PKG_HEADER_MASK;//remove starter
    pkt->cksum=pkt->id;
    for(int i=0;i<11;++i)
        pkt->cksum^=pkt->data[i];
}
void decode_uart_pkt(uart_packet* pkt){
    if(!pkt)return;
    /*pkt->arr[0]|=INTERNAL_UART_PKG_HEADER_MASK&(pkt->high_bit<<7);
    pkt->arr[1]|=INTERNAL_UART_PKG_HEADER_MASK&(pkt->high_bit<<6);
    pkt->arr[2]|=INTERNAL_UART_PKG_HEADER_MASK&(pkt->high_bit<<5);
    pkt->header&=~INTERNAL_UART_PKG_HEADER_MASK;*/
    for(int i=0;i<7;++i){
        pkt->load[i]|=((pkt->hb1>>i)&1)<<7;
    }
    pkt->load[7]|=(pkt->hb2&1)<<7;
    pkt->load[8]|=(pkt->hb2&2)<<6;
}
uint8_t check_uart_pkt(uart_packet* pkt){//if ok return false aka 0
    if(!pkt)return -1;
    //return pkt->cksum!=(pkt->header^pkt->data[0]^pkt->data[1]^pkt->data[2]^pkt->data[3]^INTERNAL_UART_PKG_HEADER_MASK);
    uint8_t res=pkt->id;
    for(int i=0;i<11;++i)
        res^=pkt->data[i];
    return res!=pkt->cksum;
}
static uart_packet send_buf;
uint8_t send_uart_pkt(uart_packet* pkt)
{
    if(!pkt)return -2;
    memcpy(&send_buf,pkt,INTERNAL_UART_PKG_SIZE);
    encode_uart_pkt(&send_buf);
    send_buf.starter=1;
    uart_send(&send_buf);
    return 0;
}
uint8_t send_uart_large(uint8_t* buf,uint8_t len,uint8_t typ)
{
    uart_packet pkg;
    memset(&pkg,0,sizeof(uart_packet));
    uint8_t id=0,res=0;
    pkg.typ=typ;
    while(len>=INTERNAL_UART_PKG_LOAD_LENGTH)
    {
        memcpy(pkg.load,buf,INTERNAL_UART_PKG_LOAD_LENGTH);
        buf+=INTERNAL_UART_PKG_LOAD_LENGTH;
        pkg.id=id++;
        //encode_uart_pkt(&pkg);
        res = send_uart_pkt(&pkg);
        if(res)
            return res;
    }
    if(len)
    {
        memset(&pkg,0,sizeof(uart_packet));
        memcpy(pkg.load,buf,len);
        pkg.id=id++;
        pkg.typ=typ;
        //encode_uart_pkt(&pkg);
        send_uart_pkt(&pkg);
    }
    return res;
}