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
    ESP_ERROR_CHECK(uart_driver_install(INTERNAL_UART_PORT_NUM, INTERNAL_UART_BUF_SIZE * 2, INTERNAL_UART_BUF_SIZE / 2, 0, NULL, 
                    INTERNAL_UART_INTR_ALLOC_FLAGS));
    ESP_ERROR_CHECK(uart_param_config(INTERNAL_UART_PORT_NUM, &uart_config));
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
    /*uint8_t* ptr=(uint8_t*)pkg;
    for(int i=0;i<6;++i){
        printf("0x%02x ",*ptr++);
    }*/
    //ESP_LOGI("uart_pkt_handler"," typ:%d id:%d\r\n",pkg->typ,pkg->id);
    //ESP_LOGI("uph","0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x",*(((uint8_t*)pkg)+0),*(((uint8_t*)pkg)+1),
    //                    *(((uint8_t*)pkg)+2),*(((uint8_t*)pkg)+3),*(((uint8_t*)pkg)+4),*(((uint8_t*)pkg)+5));
    switch(pkg->typ){
        case UART_PKG_INPUT_DATA:
            //ESP_LOGI("INPUT DATA","TYP %d",pkg->id);
            switch(pkg->id){
                case 1:
                    input_data.button_status=pkg->load;
                    break;
                case 2:
                    input_data.ljoy_status=pkg->load;
                    break;
                case 3:
                    input_data.rjoy_status=pkg->load;
                    break;
                default:
                    break;
            }
            break;
        case UART_PKG_CONNECT_CONTROL:
            switch(pkg->id)
            {
                case 0:
                    if(pkg->arr[0]){
                        //bt_status|=0x1;
                        set_bt_status(0);
                    }
                    else{
                        //bt_status&=0xfe;
                        set_bt_status(1);
                    }
                    /* reply.typ=UART_PKG_CONNECT_CONTROL;
                    reply.id=0xF;
                    reply.arr[0]=bt_status;
                    send_uart_pkt(&reply);*/
                    break;
                case 1:
                    reply.typ=UART_PKG_CONNECT_CONTROL;
                    reply.id=0;
                    memcpy(reply.arr,bt_addr,3);
                    send_uart_pkt(&reply);
                    reply.id=1;
                    memcpy(reply.arr,bt_addr+3,3);
                    send_uart_pkt(&reply);
                    break;
                case 2:
                    reply.typ=UART_PKG_CONNECT_CONTROL;
                    for(int i=0;i*3<15;++i)
                    {
                        reply.id=i+2;
                        memcpy(reply.arr,bt_hid_get_ltk()+i*3,3);
                        send_uart_pkt(&reply);
                    }
                    reply.id=5+2;
                    memset(reply.data,0,4);
                    reply.arr[0]=*(bt_hid_get_ltk()+15);
                    send_uart_pkt(&reply);
                    break;
                case 3:
                    memcpy(con_addr,pkg->arr,3);
                    break;
                case 4:
                    memcpy(con_addr+3,pkg->arr,3);
                    nvs_set("con_addr",con_addr,6);
                    con_addr_set=1;
                    reply.id=0xE;
                    send_uart_pkt(&reply);
                    break;
                case 5:
                    set_connectable();
                    break;
                default:
                    break;
            }
            break;
        case UART_PKG_TEST_ECHO:
            ESP_LOGI("UART_ECHO:","%c%c%c",pkg->arr[0],pkg->arr[1],pkg->arr[2]);
            break;
        case UART_PKG_PWR_CONTROL:
            switch(pkg->id)
            {
                case 0:
                    set_nosleep_bit(NOSLEEP_FORCED,pkg->arr[0]);
                    reply.typ=UART_PKG_PWR_CONTROL;
                    reply.id=0;
                    reply.arr[0]=pkg->arr[0];
                    send_uart_pkt(&reply);
                    break;
                case 1:
                    //uart wkup;
                    break;
            }
            break;
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
        res = uart_read_bytes(INTERNAL_UART_PORT_NUM, uart_recv_buf+len, INTERNAL_UART_BUF_SIZE-len, 0);
        //res = uart_read_bytes(INTERNAL_UART_PORT_NUM, uart_recv_buf+len, INTERNAL_UART_BUF_SIZE-len, INTERNAL_UART_GAP / portTICK_PERIOD_MS);
        if(res<0){
            ESP_LOGE(__func__,"res:%d",res);
        }
        if(res==0)continue;
        len+=res;
        if(len>100)ESP_LOGI("uart_recv","len %d",len);
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
    if(pkg)
        uart_write_bytes(INTERNAL_UART_PORT_NUM, (const char *) pkg, INTERNAL_UART_PKG_SIZE);
}

void encode_uart_pkt(uart_packet* pkt){
    if(!pkt)return;
    pkt->high_bit=(pkt->arr[0]&INTERNAL_UART_PKG_HEADER_MASK)>>7;
    pkt->high_bit|=(pkt->arr[1]&INTERNAL_UART_PKG_HEADER_MASK)>>6;
    pkt->high_bit|=(pkt->arr[2]&INTERNAL_UART_PKG_HEADER_MASK)>>5;
    pkt->load=pkt->load&INTERNAL_UART_PKG_LOAD_MASK;
    pkt->cksum=pkt->header^pkt->data[0]^pkt->data[1]^pkt->data[2]^pkt->data[3];
}
void decode_uart_pkt(uart_packet* pkt){
    if(!pkt)return;
    pkt->arr[0]|=INTERNAL_UART_PKG_HEADER_MASK&(pkt->high_bit<<7);
    pkt->arr[1]|=INTERNAL_UART_PKG_HEADER_MASK&(pkt->high_bit<<6);
    pkt->arr[2]|=INTERNAL_UART_PKG_HEADER_MASK&(pkt->high_bit<<5);
    pkt->header&=~INTERNAL_UART_PKG_HEADER_MASK;
}
uint8_t check_uart_pkt(uart_packet* pkt){//if ok return false aka 0
    if(!pkt)return -1;
    return pkt->cksum!=(pkt->header^pkt->data[0]^pkt->data[1]^pkt->data[2]^pkt->data[3]^INTERNAL_UART_PKG_HEADER_MASK);
}
static uart_packet send_buf;
uint8_t send_uart_pkt(uart_packet* pkt)
{
    if(!pkt)return -2;
    xSemaphoreTake(uart_send_mutex,portMAX_DELAY);
    memcpy(&send_buf,pkt,INTERNAL_UART_PKG_SIZE);
    encode_uart_pkt(&send_buf);
    send_buf.header|=INTERNAL_UART_PKG_HEADER_MASK;
    uart_send(&send_buf);
    xSemaphoreGive(uart_send_mutex);
    return 0;
    //return ring_buffer_push(&uart_tx_rb, (uint8_t*)&send_buf, UART_PKG_SIZE, 0);
}
uint8_t send_uart_large(uint8_t* buf,uint8_t len,uint8_t typ)
{
    uart_packet pkg;
    memset(&pkg,0,sizeof(uart_packet));
    uint8_t id=0,res=0;
    pkg.typ=typ;
    while(len>=3)
    {
        len-=3;
        memcpy(pkg.arr,buf,3);
        buf+=3;
        pkg.id=id++;
        encode_uart_pkt(&pkg);
        res = send_uart_pkt(&pkg);
        if(res)
            return res;
    }
    if(len)
    {
        memset(&pkg,0,sizeof(uart_packet));
        memcpy(pkg.arr,buf,len);
        pkg.id=id++;
        pkg.typ=typ;
        encode_uart_pkt(&pkg);
        res = send_uart_pkt(&pkg);
    }
    return res;
}