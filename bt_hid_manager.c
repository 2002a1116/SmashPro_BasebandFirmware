#include "spi_flash_mmap.h"

#include "esp_log.h"
#include "esp_hidd_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_bt.h"
#include "esp_err.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_gap_bt_api.h"
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_mac.h"
#include "esp_sleep.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/uart.h"

#include "flash.h"
#include "bt_hid_manager.h"
#include "uart.h"
#include "pwr.h"

uint32_t max(uint32_t a,uint32_t b){
        return a>b?a:b;
}
static uint8_t hid_descriptor_gamecube[] = {
0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
0x09, 0x05,        // Usage (Game Pad)
0xA1, 0x01,        // Collection (Application)
0x85, 0x21,        //   Report ID (33)
0x06, 0x01, 0xFF,  //   Usage Page (Vendor Defined 0xFF01)
0x09, 0x21,        //   Usage (0x21)
0x15, 0x00,        //   Logical Minimum (0)
0x25, 0x00,        //   Logical Maximum (0)
0x75, 0x08,        //   Report Size (8)
0x95, 0x30,        //   Report Count (48)
0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x85, 0x30,        //   Report ID (48)
0x09, 0x30,        //   Usage (0x30)
0x15, 0x00,        //   Logical Minimum (0)
0x25, 0x00,        //   Logical Maximum (0)
0x75, 0x08,        //   Report Size (8)
0x95, 0x0c,        //   Report Count (48)
0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x85, 0x3F,        //   Report ID (63)
0x05, 0x09,        //   Usage Page (Button)
0x19, 0x01,        //   Usage Minimum (0x01)
0x29, 0x10,        //   Usage Maximum (0x10)
0x15, 0x00,        //   Logical Minimum (0)
0x25, 0x01,        //   Logical Maximum (1)
0x75, 0x01,        //   Report Size (1)
0x95, 0x10,        //   Report Count (16)
0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
0x09, 0x39,        //   Usage (Hat switch)
0x15, 0x00,        //   Logical Minimum (0)
0x25, 0x07,        //   Logical Maximum (7)
0x75, 0x04,        //   Report Size (4)
0x95, 0x01,        //   Report Count (1)
0x81, 0x42,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null State)
0x75, 0x04,        //   Report Size (4)
0x95, 0x01,        //   Report Count (1)
0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x09, 0x30,        //   Usage (X)
0x09, 0x31,        //   Usage (Y)
0x09, 0x33,        //   Usage (Rx)
0x09, 0x34,        //   Usage (Ry)
0x15, 0x00,        //   Logical Minimum (0)
0x27, 0xFF, 0xFF, 0x00, 0x00,  //   Logical Maximum (65534)
0x75, 0x10,        //   Report Size (16)
0x95, 0x04,        //   Report Count (4)
0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x85, 0x01,        //   Report ID (1)
0x06, 0x01, 0xFF,  //   Usage Page (Vendor Defined 0xFF01)
0x09, 0x01,        //   Usage (0x01)
0x15, 0x00,        //   Logical Minimum (0)
0x27, 0xFF, 0xFF, 0x00, 0x00,  //   Logical Maximum (65534)
0x75, 0x08,        //   Report Size (8)
0x95, 0x30,        //   Report Count (48)
0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
0xC0               // End Collection
};
static int hid_descriptor_gc_len = sizeof(hid_descriptor_gamecube);

#define BT_HID_SEND_TIMEOUT portMAX_DELAY

static esp_hidd_app_param_t app_param;
static esp_hidd_qos_param_t both_qos;

NS_REPORT_MODE report_mode=NS_REPORT_MODE_STD;
uint8_t bt_addr[ESP_BD_ADDR_LEN];
uint8_t con_addr[ESP_BD_ADDR_LEN],con_addr_set;
uint8_t is_connected,is_paired,auto_con;
uint32_t average_packet_gap,last_packet_send,max_packet_gap,tp_timer;
static uint8_t bt_hid_send_rdy;
static QueueHandle_t report_queue = NULL;
static QueueHandle_t command_queue = NULL;
static QueueHandle_t peripheral_interrupt_queue = NULL;
static TaskHandle_t ns_bt_hid_recv_task_handle=NULL;
DRAM_ATTR peripheral_data global_input_data;
DRAM_ATTR uint8_t global_packet_timer;
static uint32_t last_packet_timer,current_packet_timer;
static SemaphoreHandle_t bt_hid_enable_semaphore=NULL;
static SemaphoreHandle_t bt_hid_send_semaphore=NULL;
static SemaphoreHandle_t bt_hid_send_std_report_semaphore=NULL;

void DRAM_ATTR (*get_peripheral_data)(peripheral_data*);
static void (*ns_bt_hid_packet_dispatch_tb[NS_PACKET_TYPE_MAX_VALUE])(cmd_packet*);
static void (*ns_cmd_subcommand_cb_tb[NS_SUBCOMMAND_ID_MAX_VLAUE])(cmd_subcommand*,uint8_t);
static uint8_t bt_hid_inited,bt_connected,try_con;
#define NS_BT_120HZ
//uint8_t IRAM_ATTR ns_bt_hid_get_packet_timer(){
inline uint8_t ns_bt_hid_get_packet_timer(){
        //return global_packet_timer;
        return xTaskGetTickCount()/5;
        //return esp_timer_get_time()/5;
}

void ns_bt_hid_register_packet_dispatch(int typ,void (*handler)(cmd_packet*)){
        if(typ>=0&&typ<NS_PACKET_TYPE_MAX_VALUE)
                ns_bt_hid_packet_dispatch_tb[typ]=handler;
}
void ns_register_subcommand_cb(int id,void (*cb)(cmd_subcommand*,uint8_t)){//SET CB NULL TO UNREG
        if(id>=0&&id<NS_SUBCOMMAND_ID_MAX_VLAUE)
                ns_cmd_subcommand_cb_tb[id]=cb;
}

void hw_send_rumble(uint8_t* buf){
        static uint8_t last_rumble[8]={0};
        static uint32_t res=0;
        static uart_packet rumble_pkt;
        memset(rumble_pkt.data,0,11);
        rumble_pkt.typ=UART_PKG_RUMBLE_FRAME;
        rumble_pkt.id=0;
        memcpy(buf,buf+4,4);//rumble always be xxx,000 or 000,xxx ,why?we just use right value now.
        memcpy(&res,buf,4);
        if(!res)return;//empty rumble,we just ignore and wait for a timeout?
        memcpy(rumble_pkt.load,buf,8);
        /*for(int i=0;i<8;++i)
        {
                if(last_rumble[i]!=buf[i])
                {
                        //ESP_LOGI(__func__,"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
                        //buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
                        //send_uart_large(buf,8,UART_PKG_RUMBLE_FRAME);
                        send_uart_pkt(&rumble_pkt);
                        memcpy(last_rumble,buf,8);
                        break;
                }
        }*/
        send_uart_pkt(&rumble_pkt);
        //send_uart_large(buf,8,UART_PKG_RUMBLE_FRAME);
}

void ns_set_peripheral_data_getter(void (*getter)(peripheral_data*)){
        get_peripheral_data=getter;
}
esp_err_t ns_bt_hid_send_raw(uint8_t typ,void* data,int len){
        return esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, typ, len, data);
}
esp_err_t ns_bt_hid_send_std_report(std_report* pkt,int len)
{
        return ns_bt_hid_send_raw(pkt->typ,((uint8_t*)pkt)+1,len+NS_STD_REPORT_BASIC_LENGTH);
}
int ns_send_report(report_packet* pkt){
        return xQueueSend(report_queue,pkt,0);
}
void ns_cmd_basic_handler(cmd_packet* pkt){
        cmd_std* cmd=(cmd_std*)pkt->cmd;
}
void ns_cmd_subcommand_dispatcher(cmd_packet* pkt){
        cmd_subcommand* cmd=(cmd_subcommand*)pkt->cmd;
        pkt->len-=NS_SUBCOMMAND_CMD_HEADER_LENGTH;
        ESP_LOGI("SUBC ","dispatch subcmd:0x%02x",cmd->subcommand_id);
        if(ns_cmd_subcommand_cb_tb[cmd->subcommand_id]){
                ns_cmd_subcommand_cb_tb[cmd->subcommand_id](cmd,pkt->len);
        }
        else if(ns_cmd_subcommand_cb_tb[0]){
                ns_cmd_subcommand_cb_tb[0](cmd,pkt->len);
        }
        else{
                ESP_LOGE("","%s error subcommand callback not exist",__func__);
        }
}
void ns_cmd_mcu_call_handler(cmd_packet* pkt){
        //todo
}
void ns_cmd_mcu_update_handler(cmd_packet* pkt){
        //todo
}
void ns_cmd_attachment_handler(cmd_packet* pkt){
        //todo
}
void ns_bt_hid_recv_task(){
        static DRAM_ATTR cmd_packet pkt;
        while(1){
                while(xQueueReceive(command_queue, &pkt, 0)){
                        //ESP_LOGI("","HID RECV");
                        if(!pkt.cmd){
                                ESP_LOGE("","%s error no cmd info",__func__);
                                continue;
                        }
                        if(pkt.cmd->typ==0x10||pkt.cmd->typ==0x01)
                                hw_send_rumble(pkt.cmd->rumble_data);//?
                        if(ns_bt_hid_packet_dispatch_tb[pkt.cmd->typ]){
                                ns_bt_hid_packet_dispatch_tb[pkt.cmd->typ](&pkt);
                        }
                        else{
                                ESP_LOGW("","%s warning unhandled hid cmd typ:%d",__func__,pkt.cmd->typ);
                        }
                        free(pkt.cmd);
                }
                taskYIELD();
                vTaskDelay(1/portTICK_PERIOD_MS);
        }
}
void update_peripheral_data(){
        if(!get_peripheral_data)
        {
               ESP_LOGE("","%s error no peripheral data getter set",__func__);
               return;
        }
        //xSemaphoreTake(update_peripheral_data_semaphore,portMAX_DELAY);
        get_peripheral_data(&global_input_data);
        //xSemaphoreGive(update_peripheral_data_semaphore);
        //input_data.rjoy_status=;
}
esp_err_t ns_send_interrupt(simple_report* rpt){
        //update_peripheral_data();
        if(!rpt)return ESP_ERR_INVALID_ARG;
        return ns_bt_hid_send_raw(rpt->typ,((uint8_t*)rpt)+1,sizeof(simple_report));
}
void rpt_warpper(std_report* rpt,std_report_data* data,uint16_t len){
        //todo : support more report typ someday
        memset(rpt,0,sizeof(std_report));
        rpt->typ=0x30;
        if(!rpt){
                ESP_LOGE("","%s error rpt not exist",__func__);
                return;
        }
        if(data&&len){
                memcpy(&(rpt->data),data,len);
                rpt->typ=0x21;
        }
        update_peripheral_data();
        rpt->input_data=global_input_data;
        //rpt->timer=global_packet_timer;
        rpt->timer=ns_bt_hid_get_packet_timer();
        rpt->battery_status=0x09;
        //todo : support battery quantity and charge state(oops,hardware doesnt support this one,so forget it);
        rpt->con_info=0x01;//todo : expect set this to 0x00 to be pro controller on battery,check if its correct;
        //rpt->rumble_status=0x80;//todo : value includes(0x70,0xC0,0xB0,0x80),what does these mean?//0x70 seems stable
        rpt->rumble_status=0xC0;
        //todo : set imu data
}
static void bt_send_enabler(void* args)
{
    bt_hid_send_rdy=is_connected;
}
void ns_bt_hid_send_task(){
        static DRAM_ATTR report_packet pkt;
        static DRAM_ATTR std_report rpt;
        static DRAM_ATTR bool act=false;
        static DRAM_ATTR uint8_t cnt=0;
        if(!is_connected)return;
        //uint8_t DRAM_ATTR empty_report;
        //while(1)
        {
                if(xQueueReceive(report_queue, &pkt, 0)){//if there is a unusual report to send,then send it
                        if(pkt.oob_data){//todo
                        }
                        else{
                                rpt_warpper(&rpt,&pkt.data,pkt.len);
                                act=true;
                        }
                }
                else{//otherwise, just send an stand report without oob data
                        rpt_warpper(&rpt,NULL,0);
                        pkt.len=0;
                }
                //for performance test with pc
                /*if(is_connected)
                        ns_bt_hid_send_std_report(&rpt,pkt.len);*/
                
                if(act||is_paired){
                        ns_bt_hid_send_std_report(&rpt,pkt.len);//send report
                        //rpt.timer++;
                        //ns_bt_hid_send_std_report(&rpt,pkt.len);//send report
                }
                else if(!cnt){
                        //empty_report[1]=global_packet_timer;
                        ns_bt_hid_get_packet_timer();
                        ns_bt_hid_send_raw(0x00,&global_packet_timer,1);
                        cnt=3;
                        //vTaskDelay(16/portTICK_PERIOD_MS);
                }
                --cnt;
                act=false;
        }
}
void print_bt_address() {
        const char* TAG = "bt_address";
        const uint8_t* bd_addr;

        //bd_addr = esp_bt_dev_get_address();
        bd_addr=bt_addr;
        ESP_LOGI(TAG, "my bluetooth address is %02X:%02X:%02X:%02X:%02X:%02X",
                 bd_addr[0], bd_addr[1], bd_addr[2], bd_addr[3], bd_addr[4], bd_addr[5]);
}
esp_err_t set_bt_address()
{
        esp_err_t err=ESP_OK;
        do{
                if(err==ESP_OK)
                        err=nvs_get("bt_addr",bt_addr,sizeof(bt_addr));
                if(err!=ESP_OK){
                        for(int i=0; i<sizeof(bt_addr); i++)
                                bt_addr[i] = esp_random()%255;
                        size_t addr_size = sizeof(bt_addr);
                        err = nvs_set("bt_addr", bt_addr, sizeof(bt_addr));
                }
                err = esp_base_mac_addr_set(bt_addr);
                printf("sat %d\n",err);
                print_bt_address();
        }while(err!=ESP_OK);
        printf("sat bt addr %d\n",err);
        return err;
}
void bt_hid_init(){
        if(!nvs_get("con_addr",con_addr,sizeof(con_addr)))
                con_addr_set=1;
        esp_err_t ret;
        bt_hid_send_semaphore=xSemaphoreCreateMutex();
        bt_hid_enable_semaphore=xSemaphoreCreateMutex();
        bt_hid_send_std_report_semaphore=xSemaphoreCreateMutex();
        report_queue=xQueueCreate(50,sizeof(report_packet));
        command_queue=xQueueCreate(50,sizeof(cmd_packet));
        peripheral_interrupt_queue=xQueueCreate(100,sizeof(uint16_t));
        ns_bt_hid_register_packet_dispatch(0x01,ns_cmd_subcommand_dispatcher);
        ns_bt_hid_register_packet_dispatch(0x03,ns_cmd_mcu_update_handler);
        ns_bt_hid_register_packet_dispatch(0x10,ns_cmd_basic_handler);
        ns_bt_hid_register_packet_dispatch(0x11,ns_cmd_mcu_call_handler);
        ns_bt_hid_register_packet_dispatch(0x12,ns_cmd_attachment_handler);
        app_param.name = "Pro Controller";
        app_param.description = "Nintendo Pro Controller";
        app_param.provider = "Nintendo";
        app_param.subclass = 0x400;
        app_param.desc_list = hid_descriptor_gamecube;
        app_param.desc_list_len = hid_descriptor_gc_len;
        memset(&both_qos, 0, sizeof(esp_hidd_qos_param_t));
        both_qos.service_type=0x01;//01:best effert 02:guaranteed
        both_qos.token_rate=7000;//bandwidth
        both_qos.token_bucket_size=0xFFFFFFFF;//maxiunm
        both_qos.peak_bandwidth=20000;//bytes/second
        both_qos.access_latency=50;//us
        both_qos.delay_variation=20000;//us
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
        //set_bt_address();
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
                ESP_LOGE("", "initialize controller failed: %s\n",  esp_err_to_name(ret));
                return;
        }
        if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
                ESP_LOGE("", "enable controller failed: %s\n",  esp_err_to_name(ret));
                return;
        }
        if ((ret = esp_bt_sleep_disable()) != ESP_OK) {
                ESP_LOGE("", "disable controller sleep fail: %s\n",  esp_err_to_name(ret));
                //return;
        }
        //esp_bt_dev_coex_status_config(ESP_BT_DEV_COEX_TYPE_BLE,ESP_BT_DEV_COEX_OP_SET,0);
        //ret = esp_bt_dev_coex_status_config(ESP_BT_DEV_COEX_TYPE_BT,ESP_BT_DEV_COEX_OP_SET,ESP_BT_DEV_COEX_BT_ST_A2DP_STREAMING);
        //ESP_LOGW(__func__,"esp_bt_dev_coex_status_config ret:%s",esp_err_to_name(ret));
        if ((ret = esp_bluedroid_init()) != ESP_OK) {
                ESP_LOGE("", "initialize bluedroid failed: %s\n",  esp_err_to_name(ret));
                return;
        }
        if ((ret = esp_bluedroid_enable()) != ESP_OK) {
                ESP_LOGE("", "enable bluedroid failed: %s\n",  esp_err_to_name(ret));
                return;
        }
        esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
        void esp_bt_gap_cb(esp_bt_gap_cb_event_t, esp_bt_gap_cb_param_t*);
        if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
                ESP_LOGE("", "gap register failed: %s\n", esp_err_to_name(ret));
                return;
        }
        uint8_t* ptr=esp_bt_dev_get_address();
        memcpy(bt_addr,ptr,ESP_BD_ADDR_LEN);//a workaround for mac changed for no reason
        print_bt_address();
        esp_bt_dev_set_device_name("Pro Controller");
        esp_bt_cod_t cod;
        esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
        esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
        cod.major = 0x05;
        cod.minor = 0x02;
        cod.service = 0x400;
        esp_bt_gap_set_cod(cod,ESP_BT_SET_COD_ALL);
        esp_bt_gap_set_security_param(param_type,&iocap,sizeof(iocap));
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        void esp_bt_hidd_cb(esp_hidd_cb_event_t, esp_hidd_cb_param_t*);
        esp_bt_hid_device_register_callback(esp_bt_hidd_cb);
        //esp_bt_hid_device_init();
        esp_bredr_tx_power_set(ESP_PWR_LVL_N12,ESP_PWR_LVL_N6);
        ESP_LOGI(__func__,"bt_hid_init1");
        if(pdPASS!=xTaskCreatePinnedToCore(ns_bt_hid_recv_task,"bt_recv_task",6144,NULL,20,&ns_bt_hid_recv_task_handle,0))
        {
                ESP_LOGE("","bt_recv_task create fail");
                return;
        }
        ESP_LOGI(__func__,"bt_hid_init2");
        //if(pdPASS!=xTaskCreatePinnedToCore(ns_bt_hid_send_task,"bt_send_task",4096,NULL,5,&ns_bt_hid_send_task_handle,0))
        //{
        //        ESP_LOGE("","bt_send_task create fail");
        //        return;
        //}
        static const esp_timer_create_args_t periodic_timer_args = {
            .callback = &ns_bt_hid_send_task,
            .name = "bt_send_timer"
        };
        ESP_LOGI(__func__,"bt_hid_init3");
        esp_timer_handle_t periodic_timer;
        ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
        #ifdef NS_BT_120HZ
        ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 8333));
        #else
        ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 16667));
        #endif
        ESP_LOGI(__func__,"BT INITED");
        esp_bt_hid_device_init();
        return;
        esp_bt_hid_device_connect(con_addr);
}
static uint8_t vc_unpluged;
void esp_bt_hidd_cb(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
        void on_hid_connection_change();
        static const char* TAG = "esp_bt_hidd_cb";
        if(event==ESP_HIDD_SEND_REPORT_EVT)return;
        //ESP_LOGI(__func__,"event %d",event);
        //uint8_t *intr_data=NULL;
        //subcommand_packet pkt;
        cmd_packet pkt;
        switch (event) {
        case ESP_HIDD_INIT_EVT:
                //xQueueReset(report_queue);
                //xQueueReset(command_queue);
                if (param->init.status == ESP_HIDD_SUCCESS) {
                        ESP_LOGI(TAG, "setting hid parameters");
                        esp_bt_hid_device_register_app(&app_param, &both_qos, &both_qos);
                } else {
                        ESP_LOGE(TAG, "init hidd failed!");
                }
                break;
        case ESP_HIDD_DEINIT_EVT:
                bt_hid_inited=0;
                break;
        case ESP_HIDD_REGISTER_APP_EVT:
                if (param->register_app.status == ESP_HIDD_SUCCESS) {
                        ESP_LOGI(TAG, "setting hid parameters success!");
                        ESP_LOGI(TAG, "setting to connectable, discoverable");
                        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                        if (param->register_app.in_use) {
                                ESP_LOGI(TAG, "start virtual cable plug!");
                                esp_bt_hid_device_connect(param->register_app.bd_addr);
                        }
                } else {
                        ESP_LOGE(TAG, "setting hid parameters failed!");
                }
                bt_hid_inited=1;
                break;
        case ESP_HIDD_UNREGISTER_APP_EVT:
                if (param->unregister_app.status == ESP_HIDD_SUCCESS) {
                        ESP_LOGI(TAG, "unregister app success!");
                } else {
                        ESP_LOGE(TAG, "unregister app failed!");
                }
                break;
        case ESP_HIDD_OPEN_EVT:
                if (param->open.status == ESP_HIDD_SUCCESS) {
                        if (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTING) {
                                ESP_LOGI(TAG, "connecting... %02x",param->open.bd_addr[0]);
                        } else if (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTED) {
                                ESP_LOGI(TAG, "connected to %02x:%02x:%02x:%02x:%02x:%02x", param->open.bd_addr[0],
                                         param->open.bd_addr[1], param->open.bd_addr[2], param->open.bd_addr[3], param->open.bd_addr[4],
                                         param->open.bd_addr[5]);
                                memcpy(con_addr,param->open.bd_addr,sizeof(con_addr));
                                esp_err_t err=nvs_set("con_addr",con_addr,sizeof(con_addr));
                                if(err)
                                        ESP_LOGE("con_addr set fail","  err:%d",err);
                                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                                bt_connected=1;
                                is_connected = true;
                        } else {
                                ESP_LOGE(TAG, "unknown connection status");
                        }
                } else {
                        ESP_LOGE(TAG, "open failed! %d %d",param->open.status,param->open.conn_status);
                }
                try_con=0;
                on_hid_connection_change();
                break;
        case ESP_HIDD_CLOSE_EVT:
                ESP_LOGW(TAG, "ESP_HIDD_CLOSE_EVT");
                if (param->close.status == ESP_HIDD_SUCCESS) {
                        if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTING) {
                                ESP_LOGE(TAG, "disconnecting...");
                        } else if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
                                ESP_LOGE(TAG, "disconnected!");
                                bt_connected=0;
                                is_connected=false;
                                is_paired=false;
                                vTaskDelay(1000/portTICK_PERIOD_MS);
                                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                        } else {
                                ESP_LOGE(TAG, "unknown connection status");
                        }
                } else {
                        ESP_LOGE(TAG, "close failed!");
                }
                on_hid_connection_change();
                break;
        case ESP_HIDD_REPORT_ERR_EVT:
                ESP_LOGI(TAG, "ESP_HIDD_REPORT_ERR_EVT");
                break;
        case ESP_HIDD_INTR_DATA_EVT:
                //ESP_LOGI(TAG, "ESP_HIDD_INTR_DATA_EVT");
                //intr_data=malloc(param->intr_data.len);
                pkt.len=param->intr_data.len;
                pkt.cmd=(cmd_std*)malloc(pkt.len+1);
                memcpy(&pkt.cmd->timer,param->intr_data.data,pkt.len);
                pkt.cmd->typ=param->intr_data.report_id;
                //ESP_LOGI(__func__,"rpt typ %d",pkt.cmd->typ);
                if(pdFAIL==xQueueSend(command_queue,&pkt,0)){
                        ESP_LOGE("","%s recv queue push fail,drop cmd",__func__);
                }
                break;
        case ESP_HIDD_VC_UNPLUG_EVT:
                ESP_LOGW(TAG, "ESP_HIDD_VC_UNPLUG_EVT");
                if (param->vc_unplug.status == ESP_HIDD_SUCCESS) {
                        if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
                                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                        } else {
                                ESP_LOGE(TAG, "unknown connection status");
                        }
                        global_packet_timer=0;
                } else {
                        ESP_LOGE(TAG, "close failed!");
                }
                bt_connected=0;
                on_hid_connection_change();
                break;
        default:
                break;
        }
}
#define SPP_TAG "esp_bt_gap_cb"
void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
        //ESP_LOGI(SPP_TAG," typ: %d",event);
        switch(event) {
        case ESP_BT_GAP_DISC_RES_EVT:
                ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_RES_EVT");
                esp_log_buffer_hex(SPP_TAG, param->disc_res.bda, ESP_BD_ADDR_LEN);
                break;
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
                ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
                break;
        case ESP_BT_GAP_RMT_SRVCS_EVT:
                ESP_LOGI(SPP_TAG, "ESP_BT_GAP_RMT_SRVCS_EVT");
                ESP_LOGI(SPP_TAG, "%d", param->rmt_srvcs.num_uuids);
                break;
        case ESP_BT_GAP_RMT_SRVC_REC_EVT:
                ESP_LOGI(SPP_TAG, "ESP_BT_GAP_RMT_SRVC_REC_EVT");
                break;
        case ESP_BT_GAP_AUTH_CMPL_EVT: {
                if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                        ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
                        esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
                } else {
                        ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
                }
                break;
        }
        case ESP_BT_GAP_MODE_CHG_EVT:
                ESP_LOGI(SPP_TAG,"ESP_BT_GAP_MODE_CHG_EVT mode:%d",param->mode_chg.mode);
                uint8_t* bd_addr = param->mode_chg.bda;
                break;
        case ESP_BT_GAP_CFM_REQ_EVT:
                ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %"PRIu32, param->cfm_req.num_val);
                esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
                break;
        default:
                break;
        }
}
#define BT_RECONNECT_TIMEOUT (6000)
#define BT_LISTENING_TIMEOUT (6000)
static uint8_t connect_fsm,reconnect_task_status;
static TaskHandle_t reconnect_task_handle=NULL; 
static uint32_t reconnect_task_tick=0;
void reconnect_task()
{
        set_nosleep_bit(NOSLEEP_BT_CONNECTING,1);
        reconnect_task_tick=xTaskGetTickCount();
        connect_fsm=0;
        do{
                if(connect_fsm){
                        vTaskDelay(200/portTICK_PERIOD_MS);
                        if(xTaskGetTickCount()-reconnect_task_tick>=BT_RECONNECT_TIMEOUT)
                                break;
                        continue;
                }else{
                        if(bt_connected){
                                reconnect_task_handle=NULL;
                                vTaskDelete(NULL);
                        }
                }
                connect_fsm=1;
                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                esp_bt_hid_device_connect(con_addr);
        }while(xTaskGetTickCount()-reconnect_task_tick<BT_RECONNECT_TIMEOUT);
        reconnect_task_handle=NULL;
        set_nosleep_bit(NOSLEEP_BT_CONNECTING,0);
        vTaskDelete(NULL);
}
void _set_connectable()
{
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_bt_hid_device_connect(con_addr);
        vTaskDelay(BT_LISTENING_TIMEOUT/portTICK_PERIOD_MS);
        reconnect_task_handle=NULL;
        set_nosleep_bit(NOSLEEP_BT_LISTENING,0);
        vTaskDelete(NULL);
}
void set_connectable()
{
        if(reconnect_task_handle)return;
        set_nosleep_bit(NOSLEEP_BT_LISTENING,1);
        if(bt_connected)set_bt_status(1);
        if(pdPASS!=xTaskCreatePinnedToCore(_set_connectable,"listen",4096,NULL,0,&reconnect_task_handle,0)){
                reconnect_task_handle=NULL;
        }
}
static uart_packet reply;
void on_hid_connection_change()
{
        ESP_LOGW(__func__,"bt_init:%d, bt_connected:%d",bt_hid_inited,bt_connected);
        if(bt_connected)
        {
                set_nosleep_bit(NOSLEEP_BT_CONNECTED,1);
                set_nosleep_bit(NOSLEEP_BT_CONNECTING,0);
                set_nosleep_bit(NOSLEEP_BT_LISTENING,0);
        }
        else{
                set_nosleep_bit(NOSLEEP_BT_CONNECTED,0);
        }
        reply.typ=UART_PKG_CONNECT_CONTROL;
        reply.id=0xF;
        reply.load[0]=bt_connected;
        send_uart_pkt(&reply);
        connect_fsm=0;
}
void connect_to_con()
{
        if(bt_connected||reconnect_task_handle)return;
        if(pdPASS!=xTaskCreatePinnedToCore(reconnect_task,"connect",4096,NULL,0,&reconnect_task_handle,0)){
                reconnect_task_handle=NULL;
        }
}
void set_bt_status(uint8_t disable)
{
        set_nosleep_bit(NOSLEEP_SET_STATUS,1);
        xQueueSemaphoreTake(bt_hid_enable_semaphore,portMAX_DELAY);
        if(disable)
        {
                if(bt_connected){
                        esp_bt_hid_device_disconnect();
                        esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                        bt_connected=0;
                }
        }
        else if(bt_hid_inited)
        {
                connect_to_con();
        }
        xSemaphoreGive(bt_hid_enable_semaphore);
        set_nosleep_bit(NOSLEEP_SET_STATUS,0);
}


esp_link_key bt_key; 
uint8_t* bt_hid_get_ltk()
{
        for(int i=0;i<16;++i)
                bt_key[i]=esp_random();
        return bt_key;
        //return esp_link_key;
}