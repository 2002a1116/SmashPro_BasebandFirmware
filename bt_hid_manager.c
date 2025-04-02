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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/periph_ctrl.h"

#include "flash.h"
#include "bt_hid_manager.h"

static uint8_t hid_descriptor_gamecube[] = {
        0x05, 0x01,    // Usage Page (Generic Desktop Ctrls)
        0x09, 0x05,    // Usage (Game Pad)
        0xA1, 0x01,    // Collection (Application)
        //Padding
        0x95, 0x03,      //     REPORT_COUNT = 3
        0x75, 0x08,      //     REPORT_SIZE = 8
        0x81, 0x03,      //     INPUT = Cnst,Var,Abs
        //Sticks
        0x09, 0x30,    //   Usage (X)
        0x09, 0x31,    //   Usage (Y)
        0x09, 0x32,    //   Usage (Z)
        0x09, 0x35,    //   Usage (Rz)
        0x15, 0x00,    //   Logical Minimum (0)
        0x26, 0xFF, 0x00, //   Logical Maximum (255)
        0x75, 0x08,    //   Report Size (8)
        0x95, 0x04,    //   Report Count (4)
        0x81, 0x02,    //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
        //DPAD
        0x09, 0x39,    //   Usage (Hat switch)
        0x15, 0x00,    //   Logical Minimum (0)
        0x25, 0x07,    //   Logical Maximum (7)
        0x35, 0x00,    //   Physical Minimum (0)
        0x46, 0x3B, 0x01, //   Physical Maximum (315)
        0x65, 0x14,    //   Unit (System: English Rotation, Length: Centimeter)
        0x75, 0x04,    //   Report Size (4)
        0x95, 0x01,    //   Report Count (1)
        0x81, 0x42,    //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null State)
        //Buttons
        0x65, 0x00,    //   Unit (None)
        0x05, 0x09,    //   Usage Page (Button)
        0x19, 0x01,    //   Usage Minimum (0x01)
        0x29, 0x0E,    //   Usage Maximum (0x0E)
        0x15, 0x00,    //   Logical Minimum (0)
        0x25, 0x01,    //   Logical Maximum (1)
        0x75, 0x01,    //   Report Size (1)
        0x95, 0x0E,    //   Report Count (14)
        0x81, 0x02,    //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
        //Padding
        0x06, 0x00, 0xFF, //   Usage Page (Vendor Defined 0xFF00)
        0x09, 0x20,    //   Usage (0x20)
        0x75, 0x06,    //   Report Size (6)
        0x95, 0x01,    //   Report Count (1)
        0x15, 0x00,    //   Logical Minimum (0)
        0x25, 0x7F,    //   Logical Maximum (127)
        0x81, 0x02,    //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
        //Triggers
        0x05, 0x01,    //   Usage Page (Generic Desktop Ctrls)
        0x09, 0x33,    //   Usage (Rx)
        0x09, 0x34,    //   Usage (Ry)
        0x15, 0x00,    //   Logical Minimum (0)
        0x26, 0xFF, 0x00, //   Logical Maximum (255)
        0x75, 0x08,    //   Report Size (8)
        0x95, 0x02,    //   Report Count (2)
        0x81, 0x02,
        0xc0
};
static int hid_descriptor_gc_len = sizeof(hid_descriptor_gamecube);

#define BT_HID_SEND_TIMEOUT portMAX_DELAY

static esp_hidd_app_param_t app_param;
static esp_hidd_qos_param_t both_qos;

NS_REPORT_MODE report_mode=NS_REPORT_MODE_STD;
uint8_t bt_addr[ESP_BD_ADDR_LEN];
uint8_t con_addr[ESP_BD_ADDR_LEN];
uint8_t is_connected,is_paired,auto_con;
uint32_t average_packet_gap,last_packet_send;
static uint8_t bt_hid_send_rdy;
static QueueHandle_t report_queue = NULL;
static QueueHandle_t command_queue = NULL;
static QueueHandle_t peripheral_interrupt_queue = NULL;
static TaskHandle_t ns_bt_hid_recv_task_handle=NULL,ns_bt_hid_send_task_handle=NULL;
static peripheral_data global_input_data;
static uint8_t global_packet_timer;
static SemaphoreHandle_t bt_hid_send_semaphore=NULL;
static SemaphoreHandle_t update_peripheral_data_semaphore=NULL;

void (*get_peripheral_data)(peripheral_data*);
static void (*ns_bt_hid_packet_dispatch_tb[NS_PACKET_TYPE_MAX_VALUE])(cmd_packet*);
static void (*ns_cmd_subcommand_cb_tb[NS_SUBCOMMAND_ID_MAX_VLAUE])(cmd_subcommand*,uint8_t);

void ns_bt_hid_register_packet_dispatch(int typ,void (*handler)(cmd_packet*)){
        if(typ>=0&&typ<NS_PACKET_TYPE_MAX_VALUE)
                ns_bt_hid_packet_dispatch_tb[typ]=handler;
}
void ns_register_subcommand_cb(int id,void (*cb)(cmd_subcommand*,uint8_t)){//SET CB NULL TO UNREG
        if(id>=0&&id<NS_SUBCOMMAND_ID_MAX_VLAUE)
                ns_cmd_subcommand_cb_tb[id]=cb;
}


void hw_send_rumble(uint32_t left,uint32_t right){

}

void ns_set_peripheral_data_getter(void (*getter)(peripheral_data*)){
        get_peripheral_data=getter;
}
esp_err_t ns_bt_hid_send_raw(void* data,int len){
        esp_err_t err;
        if(!data){
            ESP_LOGE("","%s error data addr NULL",__func__);
            return ESP_FAIL;
        }
        if(!len){
                ESP_LOGE("","%s error len == 0",__func__);
                return ESP_FAIL;
        }
        /*ESP_LOGI(__func__,"len %d",len);
        for(int i=0;i<sizeof(std_report);++i)
        {
                printf("0x%02x ",*((uint8_t*)data+i));
        }
        printf("\n");*/
        xSemaphoreTake(bt_hid_send_semaphore,BT_HID_SEND_TIMEOUT);
        err = esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1, len, data);
        if(last_packet_send)average_packet_gap=(average_packet_gap+esp_timer_get_time()-last_packet_send)>>1;
        last_packet_send=esp_timer_get_time();
        xSemaphoreGive(bt_hid_send_semaphore);
        return err;
}
esp_err_t ns_bt_hid_send_std_report(std_report* pkt,int len)
{
        if(!pkt){
                ESP_LOGE("","%s error packet addr NULL",__func__);
                return ESP_FAIL;
        }
        len+=NS_STD_REPORT_BASIC_LENGTH;
        /*if(pkt->typ==0x31);//todo
        else if(pkt->typ>=0x30){
                len=49;
        }
        else{
                len=50;
        }*/
        //if(!pkt->len)pkt->len=sizeof(std_report);
        return ns_bt_hid_send_raw(pkt,len);
}
int ns_send_report(report_packet* pkt){
        if(!pkt){
                ESP_LOGE("","%s error packet addr NULL",__func__);
                return ESP_FAIL;
        }
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
        cmd_packet pkt;
        while(1){
                if(xQueueReceive(command_queue, &pkt, portMAX_DELAY)){
                        //ESP_LOGI("","HID RECV");
                        if(!pkt.cmd){
                                ESP_LOGE("","%s error no cmd info",__func__);
                                continue;
                        }
                        hw_send_rumble(pkt.cmd->rumble_data_left,pkt.cmd->rumble_data_right);//?
                        if(ns_bt_hid_packet_dispatch_tb[pkt.cmd->typ]){
                                ns_bt_hid_packet_dispatch_tb[pkt.cmd->typ](&pkt);
                        }
                        else{
                                ESP_LOGW("","%s warning unhandled hid cmd typ:%d",__func__,pkt.cmd->typ);
                        }
                        free(pkt.cmd);
                }
        }
}
inline void update_peripheral_data(){
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
        return ns_bt_hid_send_raw(rpt,sizeof(simple_report));
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
        rpt->timer=global_packet_timer;
        rpt->battery_status=0x08;
        //todo : support battery quantity and charge state(oops,hardware doesnt support this one,so forget it);
        rpt->con_info=0x00;//todo : expect set this to 0x00 to be pro controller on battery,check if its correct;
        rpt->rumble_status=0x80;//todo : value includes(0x70,0xC0,0xB0),what does these mean?

        //todo : set imu data
}
static void bt_send_enabler(void* args)
{
    bt_hid_send_rdy=is_connected;
}
void ns_bt_hid_send_task(){
        report_packet pkt;
        std_report rpt;
        bool act=false;
        uint8_t empty_report[]={0x00,0x00};
        while(1){
                if(!bt_hid_send_rdy){
                        vTaskDelay(1/portTICK_PERIOD_MS);//sleep for 1ms to giveup timeslice
                        continue;
                }
                //ESP_LOGI("","BT SEND");
                bt_hid_send_rdy=false;
                if(xQueueReceive(report_queue, &pkt, 0)){//if there is a unusual report to send,then send it
                        if(pkt.oob_data){//todo
                        }
                        else{
                                rpt_warpper(&rpt,&pkt.data,pkt.len);
                                act=true;
                                //ns_bt_hid_send_std_report(&rpt,pkt.len);
                        }
                        /*printf("len %d ,data:",pkt.len);
                        for(int i=0;i<sizeof(std_report);++i)
                        {
                                printf("0x%02x ",*(((uint8_t*)(&rpt))+i));
                        }
                        printf("\n");*/
                }
                else{//otherwise, just send an stand report without oob data
                        rpt_warpper(&rpt,NULL,0);
                }
                if(act||is_paired)
                        ns_bt_hid_send_std_report(&rpt,pkt.len);//send report
                else{
                        empty_report[1]=global_packet_timer;
                        ns_bt_hid_send_raw(empty_report,sizeof(empty_report));
                }
               // printf("btsend \n");
                act=false;
                ++global_packet_timer;
                //ESP_LOGI("","BT SEND END");
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
        return err;
}
void bt_hid_init(){
        esp_err_t ret;
        bt_hid_send_semaphore=xSemaphoreCreateMutex();
        update_peripheral_data_semaphore=xSemaphoreCreateMutex();
        report_queue=xQueueCreate(50,sizeof(report_packet));
        command_queue=xQueueCreate(50,sizeof(cmd_packet));
        peripheral_interrupt_queue=xQueueCreate(100,sizeof(uint16_t));
        ns_bt_hid_register_packet_dispatch(0x01,ns_cmd_subcommand_dispatcher);
        ns_bt_hid_register_packet_dispatch(0x03,ns_cmd_mcu_update_handler);
        ns_bt_hid_register_packet_dispatch(0x10,ns_cmd_basic_handler);
        ns_bt_hid_register_packet_dispatch(0x11,ns_cmd_mcu_call_handler);
        ns_bt_hid_register_packet_dispatch(0x12,ns_cmd_attachment_handler);
        app_param.name = "BlueCubeMod";
        app_param.description = "BlueCubeMod Example";
        app_param.provider = "ESP32";
        app_param.subclass = 0x8;
        app_param.desc_list = hid_descriptor_gamecube;
        app_param.desc_list_len = hid_descriptor_gc_len;
        memset(&both_qos, 0, sizeof(esp_hidd_qos_param_t));
        set_bt_address();
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        esp_bt_mem_release(ESP_BT_MODE_BLE);
        if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
                ESP_LOGE("", "initialize controller failed: %s\n",  esp_err_to_name(ret));
                return;
        }
        if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
                ESP_LOGE("", "enable controller failed: %s\n",  esp_err_to_name(ret));
                return;
        }
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
        cod.major = 0x05;
        cod.minor = 0x02;
        cod.service = 1;
        esp_bt_gap_set_cod(cod,ESP_BT_SET_COD_ALL);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        void esp_bt_hidd_cb(esp_hidd_cb_event_t, esp_hidd_cb_param_t*);
        esp_bt_hid_device_register_callback(esp_bt_hidd_cb);
        esp_bt_hid_device_init();
        if(pdPASS!=xTaskCreatePinnedToCore(ns_bt_hid_recv_task,"bt_recv_task",4096,NULL,10,&ns_bt_hid_recv_task_handle,1))
        {
                ESP_LOGE("","bt_recv_task create fail");
                return;
        }
        if(pdPASS!=xTaskCreatePinnedToCore(ns_bt_hid_send_task,"bt_send_task",4096,NULL,15,&ns_bt_hid_send_task_handle,1))
        {
                ESP_LOGE("","bt_send_task create fail");
                return;
        }
        const esp_timer_create_args_t periodic_timer_args = {
            .callback = &bt_send_enabler,
            .name = "bt_send_timer"
        };
        esp_timer_handle_t periodic_timer;
        ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
        ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 8333));
}
void esp_bt_hidd_cb(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
        static const char* TAG = "esp_bt_hidd_cb";
        //uint8_t *intr_data=NULL;
        //subcommand_packet pkt;
        cmd_packet pkt;
        switch (event) {
        case ESP_HIDD_INIT_EVT:
                if (param->init.status == ESP_HIDD_SUCCESS) {
                        ESP_LOGI(TAG, "setting hid parameters");
                        esp_bt_hid_device_register_app(&app_param, &both_qos, &both_qos);
                } else {
                        ESP_LOGE(TAG, "init hidd failed!");
                }
                break;
        case ESP_HIDD_DEINIT_EVT:
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
                                ESP_LOGE(TAG, "connected to %02x:%02x:%02x:%02x:%02x:%02x", con_addr[0],
                                         con_addr[1], con_addr[2], con_addr[3], con_addr[4],con_addr[5]);
                                nvs_set("con_addr",con_addr,sizeof(con_addr));
                                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                                xQueueReset(report_queue);
                                xQueueReset(command_queue);
                                is_connected = true;
                        } else {
                                ESP_LOGE(TAG, "unknown connection status");
                        }
                } else {
                        ESP_LOGE(TAG, "open failed!");
                }
                break;
        case ESP_HIDD_CLOSE_EVT:
                ESP_LOGW(TAG, "ESP_HIDD_CLOSE_EVT");
                if (param->close.status == ESP_HIDD_SUCCESS) {
                        if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTING) {
                                ESP_LOGE(TAG, "disconnecting...");
                        } else if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
                                ESP_LOGE(TAG, "disconnected!");
                                vTaskDelay(1000/portTICK_PERIOD_MS);
                                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                                is_connected=false;
                                is_paired=false;
                        } else {
                                ESP_LOGE(TAG, "unknown connection status");
                        }
                } else {
                        ESP_LOGE(TAG, "close failed!");
                }
                break;
        case ESP_HIDD_SEND_REPORT_EVT:
                //t2=esp_timer_get_time();
                //ESP_LOGW(TAG,"timer: mills %"PRIu32,t2-t1);
                //t1=t2;
                break;
        case ESP_HIDD_REPORT_ERR_EVT:
                ESP_LOGI(TAG, "ESP_HIDD_REPORT_ERR_EVT");
                break;
        case ESP_HIDD_INTR_DATA_EVT:
                //ESP_LOGI(TAG, "ESP_HIDD_INTR_DATA_EVT");
                //intr_data=malloc(param->intr_data.len);
                pkt.len=param->intr_data.len;
                pkt.cmd=(cmd_std*)malloc(pkt.len);
                memcpy(pkt.cmd,param->intr_data.data,pkt.len);
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
                //ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_RES_EVT");
                esp_log_buffer_hex(SPP_TAG, param->disc_res.bda, ESP_BD_ADDR_LEN);
                break;
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
                //ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
                break;
        case ESP_BT_GAP_RMT_SRVCS_EVT:
                //ESP_LOGI(SPP_TAG, "ESP_BT_GAP_RMT_SRVCS_EVT");
                //ESP_LOGI(SPP_TAG, "%d", param->rmt_srvcs.num_uuids);
                break;
        case ESP_BT_GAP_RMT_SRVC_REC_EVT:
                //ESP_LOGI(SPP_TAG, "ESP_BT_GAP_RMT_SRVC_REC_EVT");
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
                //ESP_LOGI(SPP_TAG,"ESP_BT_GAP_MODE_CHG_EVT");
                uint8_t* bd_addr = param->mode_chg.bda;
                //ESP_LOGI(SPP_TAG, "oppo bluetooth address is %02X:%02X:%02X:%02X:%02X:%02X",
                //        bd_addr[0], bd_addr[1], bd_addr[2], bd_addr[3], bd_addr[4], bd_addr[5]);
                //ESP_LOGE(SPP_TAG,"BT_PM %d",param->mode_chg.mode);
                //esp_bt_hid_device_unregister_app();
                //esp_bt_hid_device_register_app(&app_param, &both_qos, &both_qos);
                break;
        case ESP_BT_GAP_CFM_REQ_EVT:
                //ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %"PRIu32, param->cfm_req.num_val);
                esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
                break;
        default:
                break;
        }
}