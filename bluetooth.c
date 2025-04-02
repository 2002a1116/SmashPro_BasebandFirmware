//
//  BlueCubeMod Firmware
//
//
//  Created by Nathan Reeves 2019
//
#include <stdio.h>
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

#include "driver/gpio.h"
#include "driver/periph_ctrl.h"

#include "bluetooth.h"
#include "flash.h"

#define LED_GPIO    25
#define PIN_SEL  (1ULL<<LED_GPIO)

#define ESP_LOGI( tag, format, ... ) 


//Buttons and sticks
static uint8_t but1_send = 0;
static uint8_t but2_send = 0;
static uint8_t but3_send = 0;
static uint8_t lx_send = 0;
static uint8_t ly_send = 0;
static uint8_t cx_send = 0;
static uint8_t cy_send = 0;
static uint8_t lt_send = 0;
static uint8_t rt_send = 0;

//static esp_hidd_callbacks_t callbacks;
static esp_hidd_app_param_t app_param;
static esp_hidd_qos_param_t both_qos;


SemaphoreHandle_t xSemaphore;
bool connected = false;
int paired = 0;
TaskHandle_t BlinkHandle = NULL;
uint8_t timer = 0;
uint8_t bt_addr[ESP_BD_ADDR_LEN];



esp_err_t my_esp_bt_hid_device_send_report(esp_hidd_report_type_t type, uint8_t id, uint16_t len, uint8_t* data)
{
        if(len>2)data[1]=timer++;
        return esp_bt_hid_device_send_report(type, id, len, data);
}




//Switch button report example //         batlvl       Buttons              Lstick           Rstick
//static uint8_t report30[] = {0x30, 0x00, 0x90,   0x00, 0x00, 0x00,   0x00, 0x00, 0x00,   0x00, 0x00, 0x00};
static uint8_t reportACK[] ={0x80,0x00,0x03};
static uint8_t report30[] = {
        0x30,
        0x0,
        0x80,
        0,//but1
        0,//but2
        0,//but3
        0,//Ls
        0,//Ls
        0,//Ls
        0,//Rs
        0,//Rs
        0,//Rs
        0x08
};
static uint8_t emptyReport[] = {
        0x0,
        0x0
};
void bt_send(uint32_t sts_button,uint32_t sts_ljoys,uint32_t sts_rjoys)
{
        esp_err_t err;
        report30[1] = timer;
        //buttons
        /*report30[3] = but1_send;
        report30[4] = but2_send;
        report30[5] = but3_send;*/
        //*(uint32_t*)(report30+3)=sts_button;
        report30[3] = sts_button;
        report30[4] = sts_button>>8;
        report30[5] = sts_button>>16;
        //encode left stick
        /*report30[6] = (lx_send << 4) & 0xF0;
        report30[7] = (lx_send & 0xF0) >> 4;
        report30[8] = ly_send;
        //encode right stick
        report30[9] = (cx_send << 4) & 0xF0;
        report30[10] = (cx_send & 0xF0) >> 4;
        report30[11] = cy_send;*/
        report30[6]=sts_ljoys;
        report30[7]=sts_ljoys>>8;
        report30[8]=sts_ljoys>>16;
        report30[9]=sts_rjoys;
        report30[10]=sts_rjoys>>8;
        report30[11]=sts_rjoys>>16;
        //xSemaphoreGive(xSemaphore);
        if(!paired)
        {
                emptyReport[1] = timer;
                my_esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1, sizeof(emptyReport), emptyReport);
                //ESP_LOGI("rp empty "," ! ");
                vTaskDelay(100/portTICK_PERIOD_MS);
        }
        else
        {
                //ESP_ERROR_CHECK(my_esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1, sizeof(report30), report30));
                err=my_esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1, sizeof(report30), report30);
                //ESP_LOGI("rp30 "," ! %d",err);
        }
        //timer+=1;
}
uint8_t hid_descriptor_gamecube[] = {
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
int hid_descriptor_gc_len = sizeof(hid_descriptor_gamecube);
///Switch Replies
//static uint8_t reply02[] = {0x21, 0x01, 0x40, 0x00, 0x00, 0x00, 0xe6, 0x27, 0x78, 0xab, 0xd7, 0x76, 0x00, 0x82, 0x02, 0x03, 0x48, 0x03, 0x02, 0xD8, 0xA0, 0x1D, 0x40, 0x15, 0x66, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                            //, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint8_t reply02[] = {0x21, 0x01, 0x40, 0x00, 0x00, 0x00, 0xe6, 0x27, 0x78, 0xab, 0xd7, 0x76, 0x00, 0x82, 0x02, 0x03, 0x48, 0x03, 0x02, 0xD8, 0xA0, 0x1D, 0x40, 0x15, 0x66, 0x01, 0x00};
static uint8_t reply08[] = {0x21, 0x02, 0x8E, 0x84, 0x00, 0x12, 0x01, 0x18, 0x80, 0x01, 0x18, 0x80, 0x80, 0x80, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                            , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint8_t reply03[] = {0x21, 0x05, 0x8E, 0x84, 0x00, 0x12, 0x01, 0x18, 0x80, 0x01, 0x18, 0x80, 0x80, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                            , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint8_t reply04[] = {0x21, 0x06, 0x8E, 0x84, 0x00, 0x12, 0x01, 0x18, 0x80, 0x01, 0x18, 0x80, 0x80, 0x83, 0x04, 0x00, 0x6a, 0x01, 0xbb, 0x01, 0x93, 0x01, 0x95, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                            , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t reply1060[] = {0x21, 0x03, 0x8E, 0x84, 0x00, 0x12, 0x01, 0x18, 0x80, 0x01, 0x18, 0x80, 0x80, 0x90, 0x10, 0x00, 0x60, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                              , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint8_t reply1050[] = { 0x21, 0x04, 0x8E, 0x84, 0x00, 0x12, 0x01, 0x18, 0x80, 0x01, 0x18, 0x80, 0x80, 0x90, 0x10, 0x50, 0x60, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                               , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint8_t reply1080[] = {0x21, 0x04, 0x8E, 0x84, 0x00, 0x12, 0x01, 0x18, 0x80, 0x01, 0x18, 0x80, 0x80, 0x90, 0x10, 0x80, 0x60, 0x00, 0x00, 0x18, 0x5e, 0x01, 0x00, 0x00, 0xf1, 0x0f,
                              0x19, 0xd0, 0x4c, 0xae, 0x40, 0xe1,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                              0x00, 0x00};
static uint8_t reply1098[] = {0x21, 0x04, 0x8E, 0x84, 0x00, 0x12, 0x01, 0x18, 0x80, 0x01, 0x18, 0x80, 0x80, 0x90, 0x10, 0x98, 0x60, 0x00, 0x00, 0x12, 0x19, 0xd0, 0x4c, 0xae, 0x40, 0xe1,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                              0x00, 0x00};
//User analog stick calib
static uint8_t reply1010[] = {0x21, 0x04, 0x8E, 0x84, 0x00, 0x12, 0x01, 0x18, 0x80, 0x01, 0x18, 0x80, 0x80, 0x90, 0x10, 0x10, 0x80, 0x00, 0x00, 0x18, 0x00, 0x00};
static uint8_t reply103D[] = {0x21, 0x05, 0x8E, 0x84, 0x00, 0x12, 0x01, 0x18, 0x80, 0x01, 0x18, 0x80, 0x80, 0x90, 0x10, 0x3D, 0x60, 0x00, 0x00, 0x19, 0xF0, 0x07, 0x7f, 0xF0, 0x07, 0x7f, 0xF0, 0x07, 0x7f, 0xF0, 0x07, 0x7f, 0xF0, 0x07, 0x7f, 0xF0, 0x07, 0x7f, 0xF0, 0x07, 0x7f, 0xF0, 0x07, 0x7f, 0x0f, 0x0f, 0x00, 0x00, 0x00, 0x00};
static uint8_t reply1020[] = {0x21, 0x04, 0x8E, 0x84, 0x00, 0x12, 0x01, 0x18, 0x80, 0x01, 0x18, 0x80, 0x80, 0x90, 0x10, 0x20, 0x60, 0x00, 0x00, 0x18, 0x00, 0x00};
static uint8_t reply4001[] = {0x21, 0x04, 0x8E, 0x84, 0x00, 0x12, 0x01, 0x18, 0x80, 0x01, 0x18, 0x80, 0x80, 0x80, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t reply4801[] = {0x21, 0x04, 0x8E, 0x84, 0x00, 0x12, 0x01, 0x18, 0x80, 0x01, 0x18, 0x80, 0x80, 0x80, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t reply3001[] = {0x21, 0x04, 0x8E, 0x84, 0x00, 0x12, 0x01, 0x18, 0x80, 0x01, 0x18, 0x80, 0x80, 0x80, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t reply3333[] = {0x21, 0x03, 0x8E, 0x84, 0x00, 0x12, 0x01, 0x18, 0x80, 0x01, 0x18, 0x80, 0x80, 0x80, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                              , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

//LED blink
void startBlink()
{
        while(1) {
                gpio_set_level(LED_GPIO, 0);
                vTaskDelay(150);
                gpio_set_level(LED_GPIO, 1);
                vTaskDelay(150);
                gpio_set_level(LED_GPIO, 0);
                vTaskDelay(150);
                gpio_set_level(LED_GPIO, 1);
                vTaskDelay(1000);
        }
        vTaskDelete(NULL);
}

esp_err_t set_bt_address()
{
        //store a random mac address in flash
        /*nvs_handle my_handle;
        esp_err_t err;
        uint8_t bt_addr[8];

        err = nvs_open("storage", NVS_READWRITE, &my_handle);
        if (err != ESP_OK) return err;

        size_t addr_size = 0;
        err = nvs_get_blob(my_handle, "mac_addr", NULL, &addr_size);
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

        if (addr_size > 0) {
                err = nvs_get_blob(my_handle, "mac_addr", bt_addr, &addr_size);
        }
        else
        {
                for(int i=0; i<8; i++)
                        bt_addr[i] = esp_random()%255;
                size_t addr_size = sizeof(bt_addr);
                err = nvs_set_blob(my_handle, "mac_addr", bt_addr, addr_size);
        }

        err = nvs_commit(my_handle);
        nvs_close(my_handle);
        esp_base_mac_addr_set(bt_addr);

        //put mac addr in switch pairing packet
        for(int z=0; z<6; z++)
                reply02[z+19] = bt_addr[z];
        return err;*/
        esp_err_t err=nvs_get("bt_mac",bt_addr,sizeof(bt_addr));
        if(err == ESP_ERR_NVS_NOT_FOUND)
        {
                for(int i=0; i<sizeof(bt_addr); i++)
                        bt_addr[i] = esp_random()%255;
                size_t addr_size = sizeof(bt_addr);
                err = nvs_set("bt_mac", bt_addr, sizeof(bt_addr));
        }
        else if(err!=ESP_OK)return err;
        for(int z=0; z<6; z++)
                reply02[z+19] = bt_addr[z];
        return err;
}
void print_bt_address() {
        const char* TAG = "bt_address";
        const uint8_t* bd_addr;

        bd_addr = esp_bt_dev_get_address();
        ESP_LOGI(TAG, "my bluetooth address is %02X:%02X:%02X:%02X:%02X:%02X",
                 bd_addr[0], bd_addr[1], bd_addr[2], bd_addr[3], bd_addr[4], bd_addr[5]);
}

#define SPP_TAG "esp_bt_gap_cb"
static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
        ESP_LOGI(SPP_TAG," typ: %d",event);
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
                ESP_LOGI(SPP_TAG,"ESP_BT_GAP_MODE_CHG_EVT");
                uint8_t* bd_addr = param->mode_chg.bda;
                ESP_LOGI(SPP_TAG, "oppo bluetooth address is %02X:%02X:%02X:%02X:%02X:%02X",
                        bd_addr[0], bd_addr[1], bd_addr[2], bd_addr[3], bd_addr[4], bd_addr[5]);
                ESP_LOGE(SPP_TAG,"BT_PM %d",param->mode_chg.mode);
                //esp_bt_hid_device_unregister_app();
                //esp_bt_hid_device_register_app(&app_param, &both_qos, &both_qos);
                break;
        case ESP_BT_GAP_CFM_REQ_EVT:
                ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %"PRIu32, param->cfm_req.num_val);
                esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
                break;
        default:
                break;
        }
}

static uint32_t t1,t2;
uint8_t con_mac[ESP_BD_ADDR_LEN];
void esp_bt_hidd_cb(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
        static const char* TAG = "esp_bt_hidd_cb";
        //if(event!=ESP_HIDD_SEND_REPORT_EVT)ESP_LOGE(TAG, " entry ! %d",event);
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
                ESP_LOGW(TAG,"open sts: %d",param->open.status);
                if (param->open.status == ESP_HIDD_SUCCESS) {
                        if (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTING) {
                                ESP_LOGI(TAG, "connecting... %02x",param->open.bd_addr[0]);
                        } else if (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTED) {
                                ESP_LOGI(TAG, "connected to %02x:%02x:%02x:%02x:%02x:%02x", param->open.bd_addr[0],
                                         param->open.bd_addr[1], param->open.bd_addr[2], param->open.bd_addr[3], param->open.bd_addr[4],
                                         param->open.bd_addr[5]);
                                memcpy(con_mac,param->open.bd_addr,sizeof(con_mac));
                                ESP_LOGE(TAG, "connected to %02x:%02x:%02x:%02x:%02x:%02x", con_mac[0],
                                         con_mac[1], con_mac[2], con_mac[3], con_mac[4],con_mac[5]);
                                ESP_LOGI(TAG, "making self non-discoverable and non-connectable.");
                                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                                ESP_LOGI(TAG, "made non-discoverable and non-connectable.");
                                xSemaphoreTake(xSemaphore, portMAX_DELAY);
                                connected = true;
                                xSemaphoreGive(xSemaphore);
                                //bt_send(0,0,0);
                        //esp_bt_hid_device_register_app(&app_param, &both_qos, &both_qos);
                                
                                //esp_bt_hid_device_connect(param->open.bd_addr);
                        } else {
                                ESP_LOGE(TAG, "unknown connection status");
                        }
                }/*else if(param->open.status == ESP_HIDD_ERROR){ 
                        ESP_LOGI(TAG, "connecting to %02x:%02x:%02x:%02x:%02x:%02x", param->open.bd_addr[0],
                                         param->open.bd_addr[1], param->open.bd_addr[2], param->open.bd_addr[3], param->open.bd_addr[4],
                                         param->open.bd_addr[5]);
                        esp_bt_hid_device_connect(param->open.bd_addr);
                        ESP_LOGI(SPP_TAG, "Failed to connect to paired switch. Setting scannable and discoverable.");
                        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                }*/ else {
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
                                xSemaphoreTake(xSemaphore, portMAX_DELAY);
                                connected = false;
                                paired=false;
                                xSemaphoreGive(xSemaphore);
                                //              esp_bt_hid_device_virtual_cable_unplug();
                                vTaskDelay(1000/portTICK_PERIOD_MS);
                                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                                /*if(nvs_get("con_mac",con_mac,sizeof(con_mac))==ESP_OK){
                                        esp_err_t ret = esp_bt_hid_device_connect(con_mac);
                                        ESP_LOGE("recon ","%d %d",con_mac[0],ret);
                                        if (ret != ESP_OK){
                                                ESP_LOGI("recon ", "Failed to connect to paired switch. Setting scannable and discoverable.");
                                                ESP_LOGE("recon ","esp_bt_hid_device_connect error %d",ret);
                                                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                                                vTaskDelay(1000/portTICK_PERIOD_MS);
                                                ESP_LOGE(TAG, "making self discoverable and connectable again 1.");
                                        }
                                        else{
                                                xSemaphoreTake(xSemaphore, portMAX_DELAY);
                                                connected = true;
                                                paired=true;
                                                xSemaphoreGive(xSemaphore);
                                        }
                                }*/
                                //start blink
                                ESP_LOGW(TAG, "disconnected 2!");
                               
                                //esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                                //vTaskDelay(1000/portTICK_PERIOD_MS);
                                ESP_LOGE(TAG, "making self discoverable and connectable again.");

                        } else {
                                ESP_LOGE(TAG, "unknown connection status");
                        }
                } else {
                        ESP_LOGE(TAG, "close failed!");
                }
                break;
        case ESP_HIDD_SEND_REPORT_EVT:
                //ESP_LOGI(TAG, "ESP_HIDD_SEND_REPORT_EVT id:0x%02x, type:%d", param->send_report.report_id,
                //         param->send_report.report_type);
                t2=esp_timer_get_time();
                //ESP_LOGW(TAG,"timer: mills %"PRIu32,t2-t1);
                t1=t2;
                break;
        case ESP_HIDD_REPORT_ERR_EVT:
                ESP_LOGI(TAG, "ESP_HIDD_REPORT_ERR_EVT");
                break;
        case ESP_HIDD_GET_REPORT_EVT:
                ESP_LOGI(TAG, "ESP_HIDD_GET_REPORT_EVT id:0x%02x, type:%d, size:%d", param->get_report.report_id,
                         param->get_report.report_type, param->get_report.buffer_size);
                break;
        case ESP_HIDD_SET_REPORT_EVT:
                ESP_LOGI(TAG, "ESP_HIDD_SET_REPORT_EVT");
                break;
        case ESP_HIDD_SET_PROTOCOL_EVT:
                ESP_LOGI(TAG, "ESP_HIDD_SET_PROTOCOL_EVT");
                if (param->set_protocol.protocol_mode == ESP_HIDD_BOOT_MODE) {
                        ESP_LOGI(TAG, "  - boot protocol");
                } else if (param->set_protocol.protocol_mode == ESP_HIDD_REPORT_MODE) {
                        ESP_LOGI(TAG, "  - report protocol");
                }
                break;
        case ESP_HIDD_INTR_DATA_EVT:
                ESP_LOGI(TAG, "ESP_HIDD_INTR_DATA_EVT");
                if(param->intr_data.len>10){
                        ESP_LOGW("SUBC: ","%x ",param->intr_data.data[10]);
                }
                        if(param->intr_data.data[10] == 2)
                        {
                                ESP_LOGW(TAG, "pairing start");
                                my_esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1, sizeof(reply02), reply02);
                        }
                        if(param->intr_data.data[10] == 8)
                        {
                                my_esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1, sizeof(reply08), reply08);
                        }
                        if(param->intr_data.data[10] == 16 && param->intr_data.data[11] == 0 && param->intr_data.data[12] == 96)
                        {
                                my_esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1, sizeof(reply1060), reply1060);
                        }
                        if(param->intr_data.data[10] == 16 && param->intr_data.data[11] == 80 && param->intr_data.data[12] == 96)
                        {
                                my_esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1, sizeof(reply1050), reply1050);
                        }
                        if(param->intr_data.data[10] == 3)
                        {
                                ESP_LOGW("set rpt mod: ", "%x ",param->intr_data.data[11]);
                                my_esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1, sizeof(reply03), reply03);
                        }
                        if(param->intr_data.data[10] == 4)
                        {
                                my_esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1, sizeof(reply04), reply04);
                        }
                        if(param->intr_data.data[10] == 16 && param->intr_data.data[11] == 128 && param->intr_data.data[12] == 96)
                        {
                                my_esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1, sizeof(reply1080), reply1080);
                        }
                        if(param->intr_data.data[10] == 16 && param->intr_data.data[11] == 152 && param->intr_data.data[12] == 96)
                        {
                                my_esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1, sizeof(reply1098), reply1098);
                        }
                        if(param->intr_data.data[10] == 16 && param->intr_data.data[11] == 16 && param->intr_data.data[12] == 128)
                        {
                                my_esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1, sizeof(reply1010), reply1010);
                        }
                        if(param->intr_data.data[10] == 16 && param->intr_data.data[11] == 61 && param->intr_data.data[12] == 96)
                        {
                                my_esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1, sizeof(reply103D), reply103D);
                        }
                        if(param->intr_data.data[10] == 16 && param->intr_data.data[11] == 32 && param->intr_data.data[12] == 96)
                        {
                                my_esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1, sizeof(reply1020), reply1020);
                        }
                        if(param->intr_data.data[10] == 64 && param->intr_data.data[11] == 1)
                        {
                                my_esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1, sizeof(reply4001), reply4001);
                        }
                        if(param->intr_data.data[10] == 72 && param->intr_data.data[11] == 1)
                        {
                                my_esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1, sizeof(reply4801), reply4801);
                        }
                        if(param->intr_data.data[10] == 48 && param->intr_data.data[11] == 1)
                        {
                                my_esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1, sizeof(reply3001), reply3001);
                                ESP_LOGW("paired","");
                                paired = 1;
                                nvs_set("con_mac",con_mac,sizeof(con_mac));
                        }
                        if(param->intr_data.data[10] == 33 && param->intr_data.data[11] == 33)
                        {
                                my_esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1, sizeof(reply3333), reply3333);
                                
                        }
                        if(param->intr_data.data[10] == 64 && param->intr_data.data[11] == 2)
                        {
                                my_esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1, sizeof(reply4001), reply4001);
                        }
                        ESP_LOGW(TAG, "got an interrupt report from host, subcommand: %d  %d  %d Length: %d", param->intr_data.data[10], param->intr_data.data[11], param->intr_data.data[12], param->intr_data.len);
                }
                else
                {
                        ESP_LOGI("heap size:", "%d", xPortGetFreeHeapSize());
                        ESP_LOGI(TAG, "pairing packet size != 49, subcommand: %d  %d  %d  Length: %d", param->intr_data.data[10], param->intr_data.data[11], param->intr_data.data[12], param->intr_data.len);
                }
                break;
        case ESP_HIDD_VC_UNPLUG_EVT:
                ESP_LOGW(TAG, "ESP_HIDD_VC_UNPLUG_EVT");
                if (param->vc_unplug.status == ESP_HIDD_SUCCESS) {
                        if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
                                
                                ESP_LOGI(TAG, "disconnected!");

                                ESP_LOGI(TAG, "making self discoverable and connectable again.");
                                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                        } else {
                                ESP_LOGE(TAG, "unknown connection status");
                        }
                        timer=0;
                } else {
                        ESP_LOGE(TAG, "close failed!");
                }
                break;
        default:
                break;
        }
}


esp_vhci_host_callback_t vhci_cb;
void notify_host_send_available(void){
        //do nothing
}
int notify_host_recv(uint8_t *data, uint16_t len){
        //hci packet recv
        ESP_LOGW("hci cb: ","len:%d,  typ:%x,  opcode:0x%04x",len,data[0],*(uint16_t*)(data+1));
        return len;
}


void bt_init(void)
{
        //flash LED
        vTaskDelay(100);
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(100);
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(100);
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(100);
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(100);
        gpio_set_level(LED_GPIO, 0);
        const char* TAG = "app_main";
        esp_err_t ret;

        xSemaphore = xSemaphoreCreateMutex();

        app_param.name = "BlueCubeMod";
        app_param.description = "BlueCubeMod Example";
        app_param.provider = "ESP32";
        app_param.subclass = 0x8;
        app_param.desc_list = hid_descriptor_gamecube;
        app_param.desc_list_len = hid_descriptor_gc_len;
        memset(&both_qos, 0, sizeof(esp_hidd_qos_param_t));

        /*ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
                ESP_ERROR_CHECK(nvs_flash_erase());
                ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK( ret );*/

        set_bt_address();

        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        esp_bt_mem_release(ESP_BT_MODE_BLE);
        if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
                ESP_LOGE(TAG, "initialize controller failed: %s\n",  esp_err_to_name(ret));
                return;
        }

        if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
                ESP_LOGE(TAG, "enable controller failed: %s\n",  esp_err_to_name(ret));
                return;
        }

        if ((ret = esp_bluedroid_init()) != ESP_OK) {
                ESP_LOGE(TAG, "initialize bluedroid failed: %s\n",  esp_err_to_name(ret));
                return;
        }

        if ((ret = esp_bluedroid_enable()) != ESP_OK) {
                ESP_LOGE(TAG, "enable bluedroid failed: %s\n",  esp_err_to_name(ret));
                return;
        }
        esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

        if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
                ESP_LOGE(TAG, "gap register failed: %s\n", esp_err_to_name(ret));
                return;
        }

        ESP_LOGI(TAG, "setting device name");
        esp_bt_dev_set_device_name("Pro Controller");

        ESP_LOGI(TAG, "setting cod major, Pro Controller");
        esp_bt_cod_t cod;
        cod.major = 0x05;
        cod.minor = 0b0000010;
        cod.service = 1;
        //esp_bt_gap_set_cod(cod,ESP_BT_SET_COD_MAJOR_MINOR     );
        esp_bt_gap_set_cod(cod,ESP_BT_SET_COD_ALL);


        vTaskDelay(2000 / portTICK_PERIOD_MS);

        ESP_LOGI(TAG, "register hid device callback");
        esp_bt_hid_device_register_callback(esp_bt_hidd_cb);

        ESP_LOGI(TAG, "starting hid device");
        esp_bt_hid_device_init();

        vhci_cb.notify_host_recv=notify_host_recv;
        vhci_cb.notify_host_send_available=notify_host_send_available;
        //if((ret=esp_vhci_host_register_callback(&vhci_cb))!=ESP_OK){
        //        ESP_LOGE(TAG, "vhci cb init fail");
        //}

        /*do{
                                //esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                                ESP_LOGE("RECON ","START 1");
                                vTaskDelay(3000/portTICK_PERIOD_MS);
                                ESP_LOGE("RECON ","START 2");
                                //connected=1;
                                if(nvs_get("con_mac",con_mac,sizeof(con_mac))==ESP_OK){
                                        esp_err_t ret = esp_bt_hid_device_connect(con_mac);
                                        ESP_LOGE("recon ","%d %d",con_mac[0],ret);
                                        if (ret != ESP_OK){
                                                ESP_LOGI("recon ", "Failed to connect to paired switch. Setting scannable and discoverable.");
                                                ESP_LOGE("recon ","esp_bt_hid_device_connect error %d",ret);
                                                //esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                                        }
                                }
        }while(0);*/
       
        //start blinking
       // xTaskCreate(recon, "blink_task", 1024, NULL, 1, &BlinkHandle);

        print_bt_address();
}