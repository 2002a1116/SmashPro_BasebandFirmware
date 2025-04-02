#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "driver/uart.h"

#include "ananlog.h"
#include "wifi.h"
#include "hardware.h"
#include "macro.h"
#include "flash.h"
#include "uart.h"

#include "bt_hid_manager.h"
#include "ns_com.h"
#include "tcp_com.h"
#include "web_ota.h"
#include "pwr.h"

static uint32_t sts_button;
static uint16_t sts_button_simple;
static uint32_t adc_data[10];
static uint32_t t1,t2;
static simple_report simple_rpt;
TaskHandle_t MonitorHandle = NULL,ananlog_read_task=NULL,uart_heart_beat_task_handle=NULL;
TaskHandle_t fliper_task;
uint8_t fliper_enable=0,cnt=0;
void fliper(){
    while(1){
        vTaskDelay(15000/portTICK_PERIOD_MS);
        fliper_enable=!fliper_enable;
        max_packet_gap=0;
        sts_button=0;
        //if(!fliper_enable)continue;
        //ESP_LOGI(__func__,"  std: %d ",(int)sts_button);
        //sts_button=sts_button^(1<<0);
        //sts_button=(sts_button^(sts_button&0b10))|((!(sts_button&1))<<1);
        //sts_button=sts_button<<1;
    }
}

void IRAM_ATTR ananlog_update(){
    while(1){
        ananlog_read(adc_data,sizeof(adc_data));
        vTaskDelay(8/portTICK_PERIOD_MS);
    }
}
void IRAM_ATTR button_upd_all(){
    sts_button=0;
    for(int i=0;i<BT_BUTTON_MAX_CNT;++i){
        if(bt_num_to_gpio_num(i)<GPIO_NUM_MAX_VALUE)
            sts_button|=((!gpio_get_level(bt_num_to_gpio_num(i)))<<i);
    }
}
void IRAM_ATTR get_peripheral_data_handler(peripheral_data* data){
    if(!data){
        ESP_LOGE("","%s error peripheral_data addr is NULL",__func__);
        return;
    }
    ananlog_read(adc_data,sizeof(adc_data));
    button_upd_all();
    data->ljoy_status=~((adc_data[ADC_CHANNEL_LJOYS_VERT]<<12)+adc_data[ADC_CHANNEL_LJOYS_HORI]);
    data->rjoy_status=((adc_data[ADC_CHANNEL_RJOYS_VERT]<<12)+4095-adc_data[ADC_CHANNEL_RJOYS_HORI]);
    data->button_status=sts_button;
    if(fliper_enable){
        ++cnt;
        if(cnt>3){
            cnt=0;
        }
        sts_button=(1<<(cnt+BT_BUTTON_DOWN));
        data->button_status=sts_button;
    }
}
void gpio_update_cb(){
    ESP_LOGI("","send interrupt");
    simple_rpt.typ=0x3F;
    simple_rpt.stick_hat_data=0x08;//? todo : figure out whats this
    simple_rpt.button_status=sts_button_simple;
    simple_rpt.ljoy_status_hori=4096-adc_data[ADC_CHANNEL_LJOYS_HORI];
    simple_rpt.ljoy_status_vert=4096-adc_data[ADC_CHANNEL_LJOYS_VERT];
    simple_rpt.rjoy_status_hori=adc_data[ADC_CHANNEL_RJOYS_HORI];
    simple_rpt.rjoy_status_vert=adc_data[ADC_CHANNEL_RJOYS_VERT];
    simple_rpt.rjoy_status_hori=2048;
    simple_rpt.rjoy_status_vert=2048;
    //ns_send_interrupt(&simple_rpt);
}
static void set_button_status(uint8_t gpio_num,uint8_t v){
    sts_button=((sts_button&BT_NUM_TO_BIT(gpio_num_to_bt_num(gpio_num)))^sts_button)|BT_VALUE_TO_BIT(v,gpio_num_to_bt_num(gpio_num));
    sts_button_simple=((sts_button_simple&BT_NUM_TO_BIT(gpio_num_to_bt_num_simple(gpio_num)))^sts_button_simple)|BT_VALUE_TO_BIT(v,gpio_num_to_bt_num_simple(gpio_num));
}
void gpio_isr_cb(uint8_t gpio_num,uint8_t v)
{
    v=!v;
    set_button_status(gpio_num,v);
    //io_num=gpio_num_to_bt_num(io_num);
    printf("set but:%"PRIu8" ,value:%"PRIu8"\n", gpio_num,v);
}
static void monitor(){
    static size_t buflen=0;
    static uint8_t value=0;
        ESP_LOGE("Monitor","enabled");
    while(1){
        uart_get_buffered_data_len(INTERNAL_UART_PORT_NUM,&buflen);
        ESP_LOGI("Monitor","sts: %d %d %"PRIu32" %d %d %d",
        is_connected,is_paired,average_packet_gap,global_packet_timer,buflen,gpio_get_level(GPIO_NUM_21));
        /*for(int i=0;i<24;++i){
            //printf("%d",(int)(sts_button>>i)&1);
            printf("%d",(int)(global_input_data.button_status>>i)&1);
        }*/
       printf("%d %d %d",(global_input_data.button_status>>16)&0xff,
       (global_input_data.button_status>>8)&0xff,global_input_data.button_status&0xff);
        printf("\n");
        printf("%d %d %d %d\n",(global_input_data.ljoy_status&0xfff),global_input_data.ljoy_status>>12,
        (global_input_data.rjoy_status&0xfff),global_input_data.rjoy_status>>12);
        //ESP_LOGI("Monitor","button sts:%032b",(unsigned int)sts_button);
       // ESP_LOGI("Monitor","paired %d",is_paired);
        //ESP_LOGI("Monitor","average gap %"PRIu32,average_packet_gap);
        vTaskDelay(2000/portTICK_PERIOD_MS);
    }
}

//todo 24.9.27 rewrite entire tcp com,make cmd be like linux bash or windows cmd--> {prog (param)},'\0' as the end of the cmd(cmd will be a c style string)
void tcp_cmd_echo(char* param,int len){
    if(len)
        ESP_LOGI(__func__,"len:%d ,info:%s",len,param);
    tcp_send(param,len);
}
void tcp_cmd_set_button_debounce_time(char* param,int len){
    uint8_t x=atoi(param);
    if(x<4){
        ESP_LOGE(__func__,"error debounce time too short time:%d",x);
        return;
    }
    gpio_set_debounce_time(x);
}
void tcp_cmd_set_button_status(char* param,int len){
    uint32_t p1;
    int gpio_num,v;
    p1=str_find_char(param,len,' ');
    if(p1+1>=len){
        ESP_LOGW(__func__,"error cant decode param");
        return;
    }
    gpio_num=atoi(param);
    v=atoi(param+p1+1);
    set_button_status(gpio_num,v);
    ESP_LOGI(__func__,"set button status %d %d",gpio_num,v);
}
void tcp_cmd_flip_enabler(char* param,int len)
{
    uint8_t x=atoi(param);
    fliper_enable=x;
    vTaskDelay(10/portTICK_PERIOD_MS);
    sts_button=0;
    max_packet_gap=0;
}
void tcp_cmd_get_battery_voltage(char* param,int len){
    char buf[15]="&BAT_VOLT ";
    sprintf(buf+10,"%d",(int)adc_data[ADC_CHANNEL_BAT_VOLT]);
    tcp_send(buf,sizeof(buf));
}
void on_tcp_recv(char* pkt,int len,void* context,int context_len){
    tcp_cmd_dispatcher(pkt,len);
}
void tcp_cmd_handler_init()
{
    tcp_cmd_register_handler("#set_button",sizeof("#set_button")-1,tcp_cmd_set_button_status);
    tcp_cmd_register_handler("echo",sizeof("echo")-1,tcp_cmd_echo);
    tcp_cmd_register_handler("set_debounce",sizeof("set_debounce")-1,tcp_cmd_set_button_debounce_time);
    //test beneth
    tcp_cmd_register_handler("$fliper_enable",sizeof("$fliper_enable")-1,tcp_cmd_flip_enabler);
    tcp_cmd_register_handler("$get_battery_voltage",sizeof("$get_battery_voltage")-1,tcp_cmd_get_battery_voltage);
}
#define GPIO_CTRL_CHBOOT0_NUM (GPIO_NUM_21)
void ctrl_gpio_init()
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = GPIO_PIN(GPIO_CTRL_CHBOOT0_NUM);
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_CTRL_CHBOOT0_NUM,0);
}
void uart_heart_beat_task()
{
    static uart_packet pkt;
    pkt.typ=UART_PKG_CONNECT_CONTROL;
    pkt.id=0xC;
    while(1)
    {
        pkt.arr[0]=is_paired;
        //ESP_LOGI("","UART SEND STARTER");
        send_uart_pkt(&pkt);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
TaskHandle_t general_monitor_handle=NULL;
void general_monitor()
{
    while(1)
    {
        set_disablesleep_bit(1,!gpio_get_level(GPIO_NUM_32));
        #ifdef DEBUG_NONSLEEP
            set_disablesleep_bit(7,1);
        #endif
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
void general_monitor_init(){
    gpio_config_t io_conf = {};
    //interrupt of rising edge
    //io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_PIN(GPIO_NUM_32);
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-down mode
    //io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}
void app_main(void)
{
    firmware_Sha256();
    //ctrl_gpio_init();
    //while(1)vTaskDelay(1000/portTICK_PERIOD_MS);
    //input_evt_queue = xQueueCreate(20, sizeof(uint32_t));
    vTaskDelay(2000/portTICK_PERIOD_MS);
    //delay the init
    memset(adc_data, -1, sizeof(adc_data));
    esp_err_t ret=nvs_init();
    ESP_LOGI("nvs init","ret :%d",ret);
    vTaskDelay(2000/portTICK_PERIOD_MS);
    if(!nvs_get("con_addr",con_addr,6)){
        con_addr_set=1;
        ESP_LOGE(__func__, "con addr %02x:%02x:%02x:%02x:%02x:%02x", con_addr[0],
        con_addr[1], con_addr[2], con_addr[3], con_addr[4],con_addr[5]);
    }
    ESP_ERROR_CHECK(ret);
    //tcp_cmd_handler_init();
    wifi_init(on_tcp_recv,NULL,0);
    ns_subcommand_callback_init();
    ns_set_peripheral_data_getter(uart_get_peripheral_data);
    //ns_set_peripheral_data_getter(get_peripheral_data_handler);
    bt_hid_init();
    uart_init();
    xTaskCreatePinnedToCore(monitor,"monitor",2048,NULL,0,&MonitorHandle,0);
    xTaskCreatePinnedToCore(uart_heart_beat_task,"uart_hb",2048,NULL,0,&uart_heart_beat_task_handle,0);
    xTaskCreatePinnedToCore(general_monitor,"gm",1024,NULL,0,&general_monitor_handle,0);
    xTaskCreatePinnedToCore(enter_sleep_task,"sleep",3072,NULL,0,&enter_sleep_task_handle,0);
    HttpOTA_server_init();

    //send_uart_pkt(&pkt);
}