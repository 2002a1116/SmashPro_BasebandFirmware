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

#include "flash.h"
#include "bt_hid_manager.h"
#include "ns_com.h"
#include "uart.h"


#define NS_SUBCOMMAND_CB_PARAM cmd_subcommand* cmd,uint8_t len
#define DEFAULT_ACK NS_SUBCOMMAND_STATUS_ACK

#define SPI_DATA_MAX_SIZE (30)
report_packet pkt;

#define SUBC_REPORT_BASIC_LENGTH (2)
inline void pkt_clr(){
    pkt.len=pkt.oob_len=0;
    pkt.oob_data=NULL;
}
void _ns_subcommand_set_ack(uint8_t id,uint8_t status){//default status 0x80
    pkt.data.subcommand_report.subcommand_status=status;
    //pkt.data.subcommand_report.subcommand_id=cmd->subcommand_id;
    pkt.data.subcommand_report.subcommand_id=id;
}

#define SUBC_ID_DEFAULT (0x00)
#define SUBC_REPORT_DEFAUT_LENGTH (3)
void ns_subcommand_default(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    _ns_subcommand_set_ack(cmd->subcommand_id,DEFAULT_ACK);
    pkt.data.subcommand_report.subcommand_data[0]=0x03;
    pkt.len=SUBC_REPORT_DEFAUT_LENGTH;
    ns_send_report(&pkt);
}

#define SUBC_REPORT_GET_DEVICE_INFO_LENGTH (14)
#define SUBC_ID_GET_DEVICE_INFO (0x02)
#pragma pack(push,1)
typedef struct{
    uint8_t fw_version_major;
    uint8_t fw_version_minor;
    uint8_t device_typ;
    uint8_t reserved_0x02;
    uint8_t addr[ESP_BD_ADDR_LEN];
    uint8_t reserved_0x01;
    uint8_t use_spi_color;
}device_info;
#pragma pack(pop)
device_info default_device_info={0x03,0x48,0x03,0x02,{},0x01,0x00};
void ns_subcommand_get_device_info(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    //pkt.report.subcommand_report.subcommand_status=NS_SUBCOMMAND_STATUS_ACK;
    //pkt.report.subcommand_report.subcommand_id=cmd->subcommand_id;
    //_ns_subcommand_set_ack(cmd->subcommand_id,DEFAULT_ACK);
    _ns_subcommand_set_ack(cmd->subcommand_id,0x82);
    uint8_t* addr = esp_bt_dev_get_address();
    memcpy(default_device_info.addr,addr,ESP_BD_ADDR_LEN);
    //ESP_LOGW("MAC ","0x%02x  0x%02x  0x%02x",default_device_info.addr[5],bt_addr[5],addr[5]);
    memcpy(pkt.data.subcommand_report.subcommand_data,&default_device_info,sizeof(device_info));
    pkt.len=SUBC_REPORT_GET_DEVICE_INFO_LENGTH;
    ns_send_report(&pkt);
}


#define SUBC_ID_SET_INPUT_MODE (0x03)
void ns_subcommand_set_input_mode(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    _ns_subcommand_set_ack(cmd->subcommand_id,DEFAULT_ACK);
    pkt.len=SUBC_REPORT_BASIC_LENGTH;
    ns_send_report(&pkt);
    ESP_LOGW("","%s set input mode %02x",__func__,cmd->subcommand_data[0]);
    //is_paired=true;
}

#define SUBC_REPORT_TRIGGER_BUTTONS_ELAPSED_TIME_LENGTH (16)
#define SUBC_ID_TRIGGER_BUTTONS_ELAPSED_TIME (0x04)
#pragma pack(push,1)
typedef struct{
    uint16_t L;
    uint16_t R;
    uint16_t ZL;
    uint16_t ZR;
    uint16_t SL;
    uint16_t SR;
    uint16_t HOME;
}buttons_elapsed_time;
#pragma pack(pop)
buttons_elapsed_time default_et={0x6a00,0xbb01,0x9301,0x9501,0x00};
void ns_subcommand_trigger_buttons_elapsed_time(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    //_ns_subcommand_set_ack(cmd->subcommand_id,DEFAULT_ACK);
    _ns_subcommand_set_ack(cmd->subcommand_id,0x83);
    buttons_elapsed_time* ptr=(buttons_elapsed_time*)pkt.data.subcommand_report.subcommand_data;
    *ptr=default_et;
    //todo
    pkt.len=SUBC_REPORT_TRIGGER_BUTTONS_ELAPSED_TIME_LENGTH;
    ns_send_report(&pkt);
}

#define SUBC_REPORT_GET_PAGE_LIST_STATE_LENGTH (3)
#define SUBC_ID_GET_PAGE_LIST_STATE (0x05)
void ns_subcommand_get_page_list_state(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    _ns_subcommand_set_ack(cmd->subcommand_id,DEFAULT_ACK);
    pkt.data.subcommand_report.subcommand_data[0]=0x1;
    pkt.len=SUBC_REPORT_GET_PAGE_LIST_STATE_LENGTH;
    ns_send_report(&pkt);
}

#define SUBC_ID_SET_HCI_STATE (0x06)
void ns_subcommand_set_hci_state(NS_SUBCOMMAND_CB_PARAM){
    uint8_t state=cmd->subcommand_data[0];
    //todo
    switch(state){
        case 0b0://break connection,go page scan
            break;
        case 0b1://reboot,go page mode
            break;
        case 0b10://reboot,go inquiry mode
            break;
        case 0b100://reboot,go reconnect
            break;
    }
}

#define SUBC_ID_RESET_PAIRING_INFO (0x07)
void ns_subcommand_reset_pairing_info(NS_SUBCOMMAND_CB_PARAM){
    //todo remove con_mac
}

#define SUBC_ID_SET_SHIPPING_MODE (0x08)
void ns_subcommand_set_shipping_mode(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    _ns_subcommand_set_ack(cmd->subcommand_id,DEFAULT_ACK);
    pkt.len=SUBC_REPORT_BASIC_LENGTH;
    ns_send_report(&pkt);
    //this subcommand will be send by switch on every connection,so do nothing for now

}

static uint8_t reply1060[] = {0x00, 0x60, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                              , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint8_t reply1050[] = {  0x50, 0x60, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                               , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint8_t reply1080[] = { 0x80, 0x60, 0x00, 0x00, 0x18, 0x5e, 0x01, 0x00, 0x00, 0xf1, 0x0f,
                              0x19, 0xd0, 0x4c, 0xae, 0x40, 0xe1,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                              0x00, 0x00};
static uint8_t reply1098[] = { 0x98, 0x60, 0x00, 0x00, 0x12, 0x19, 0xd0, 0x4c, 0xae, 0x40, 0xe1,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                              0x00, 0x00};
//User analog stick calib
static uint8_t reply1010[] = { 0x10, 0x80, 0x00, 0x00, 0x18, 0x00, 0x00};
static uint8_t reply103D[] = {0x3D, 0x60, 0x00, 0x00, 0x19, 0xF0, 0x07, 0x7f, 0xF0, 0x07, 0x7f, 0xF0, 0x07, 0x7f, 0xF0, 0x07, 0x7f, 0xF0, 0x07, 0x7f, 0xF0, 0x07, 0x7f, 0xF0, 0x07, 0x7f, 0xF0, 0x07, 0x7f, 0x0f, 0x0f, 0x00, 0x00, 0x00, 0x00};
static uint8_t reply1020[] = {0x20, 0x60, 0x00, 0x00, 0x18, 0x00, 0x00};

#define SUBC_ID_SPI_READ (0x10)
#define SUBC_REPORT_SPI_READ_LENGTH (7)
#define SUBC_INPUT_SPI_READ_DATA_LENGTH (5)
void ns_subcommand_spi_read(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    uint32_t addr=*(uint32_t*)cmd->subcommand_data;
    uint8_t size=cmd->subcommand_data[4];
    ESP_LOGI("SPI_READ","32addr %"PRIu32" :0x%02x,0x%02x,0x%02x,0x%02x, size:0x%02x",addr,cmd->subcommand_data[0],cmd->subcommand_data[1],cmd->subcommand_data[2],cmd->subcommand_data[3],size);
    if(size>0x1D){
        ESP_LOGW("","%s warning spi read too long addr:%"PRIu32"  ,size:%"PRIu8,__func__,addr,size);
    }
    _ns_subcommand_set_ack(cmd->subcommand_id,0x90);
    memcpy(pkt.data.subcommand_report.subcommand_data,cmd->subcommand_data,SUBC_INPUT_SPI_READ_DATA_LENGTH);
    //todo actual spi read
    int typ=0;
    switch(*(uint16_t*)cmd->subcommand_data)//todo : rewrite these shit,figure out whats what
    {
        case 0x6000:
            typ=0;
            size=sizeof(reply1060);
            memcpy(pkt.data.subcommand_report.subcommand_data,reply1060,sizeof(reply1060));
            break;
        case 0x6050:
            //pkt.data.subcommand_report.subcommand_data[4]=0x18;
            typ=1;
            size=sizeof(reply1050);
            memcpy(pkt.data.subcommand_report.subcommand_data,reply1050,sizeof(reply1050));
            break;
        case 0x6080:
            typ=2;
            size=sizeof(reply1080);
            memcpy(pkt.data.subcommand_report.subcommand_data,reply1080,sizeof(reply1080));
            break;
        case 0x6098:
            typ=3;
            size=sizeof(reply1098);
            memcpy(pkt.data.subcommand_report.subcommand_data,reply1098,sizeof(reply1098));
            break;
        case 0x8010:
            typ=4;
            size=sizeof(reply1010);
            memcpy(pkt.data.subcommand_report.subcommand_data,reply1010,sizeof(reply1010));
            break;
        case 0x603d:
            typ=5;
            size=sizeof(reply103D);
            memcpy(pkt.data.subcommand_report.subcommand_data,reply103D,sizeof(reply103D));
            break;
        case 0x6020:
            typ=6;
            size=sizeof(reply1020);
            memcpy(pkt.data.subcommand_report.subcommand_data,reply1020,sizeof(reply1020));
            break;
        default:
            ESP_LOGW("","warning spi read unknown addr");
            break;
    }
    //pkt.len=SUBC_REPORT_SPI_READ_LENGTH+size;
    pkt.len=SUBC_REPORT_BASIC_LENGTH+size;
    //memcpy(pkt.data.subcommand_report.subcommand_data+SUBC_INPUT_SPI_READ_DATA_LENGTH,spi_data[typ],size);
    ESP_LOGW("","len :%d ,typ:%d",pkt.len,typ);
    ns_send_report(&pkt);
}

#define SUBC_REPORT_SPI_WRITE_LENGTH (3)
#define SUBC_ID_SPI_WRITE (0x11)
void ns_subcommand_spi_write(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    uint32_t addr=*(uint32_t*)cmd->subcommand_data;
    uint8_t size=cmd->subcommand_data[4];
    uint8_t *data=cmd->subcommand_data+5;
    //todo spi write,Replies with x8011 ack and a uint8 status. x00 = success, x01 = write protected.
    _ns_subcommand_set_ack(cmd->subcommand_id,DEFAULT_ACK);
    pkt.data.subcommand_report.subcommand_data[0]=0x00;
    pkt.len=SUBC_REPORT_SPI_WRITE_LENGTH;
    ns_send_report(&pkt);
}

#define SUBC_REPORT_SPI_ERASE_LENGTH (3)
#define SUBC_ID_SPI_ERASE (0x12)
void ns_subcommand_spi_erase(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    uint32_t addr=*(uint32_t*)cmd->subcommand_data;
    uint16_t size=4096;
    //todo spi erase
    //Erases the whole 4KB in the specified address to 0xFF.
    //Replies with x8012 ack and a uint8 status. x00 = success, x01 = write protected.
    _ns_subcommand_set_ack(cmd->subcommand_id,DEFAULT_ACK);
    pkt.data.subcommand_report.subcommand_data[0]=0x00;
    pkt.len=SUBC_REPORT_SPI_ERASE_LENGTH;
    ns_send_report(&pkt);
}

#define SUBC_ID_RESET_MCU (0x20)
void ns_subcommand_reset_mcu(NS_SUBCOMMAND_CB_PARAM){
    //do nothing for now
}

#define SUBC_ID_SET_MCU_CONF (0x21)
#define SUBC_REPORT_SET_MCU_CONF_LENGTH (36)
void ns_subcommand_set_mcu_conf(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    //Takes 38 or 37 bytes long argument data.
    ESP_LOGI("","%s set mcu conf length:%d",__func__,len);
    //reply ack with 0xA0 0x20??? why not 0x21?
    //todo : check if its correct
    pkt.data.subcommand_report.subcommand_id=0x21;
    pkt.data.subcommand_report.subcommand_status=0xA0;
    memcpy(pkt.data.subcommand_report.subcommand_data,cmd->subcommand_data,NS_SUBCOMMAND_REPORT_DATA_MAX_LENGTH);
    pkt.len=SUBC_REPORT_SET_MCU_CONF_LENGTH;
    ns_send_report(&pkt);
}

#define SUBC_ID_SET_MCU_STATE (0x22)
//todo : no reply? check if its correct
void ns_subcommand_set_mcu_state(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    uint8_t state=cmd->subcommand_data[0];
    _ns_subcommand_set_ack(cmd->subcommand_id,DEFAULT_ACK);
    pkt.len=SUBC_REPORT_BASIC_LENGTH;
    ns_send_report(&pkt);
    //0:suspend 1:resume 2:resume for update
}

#define SUBC_ID_ENABLE_MCU_POLLING (0x24)
#define SUBC_REPORT_ENABLE_MCU_POLLING_LENGTH (3)
void ns_subcommand_enable_mcu_polling(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    //todo
    //ans with ack always for now;
    _ns_subcommand_set_ack(cmd->subcommand_id,DEFAULT_ACK);
    pkt.data.subcommand_report.subcommand_data[0]=0x00;
    pkt.len=SUBC_REPORT_ENABLE_MCU_POLLING_LENGTH;
    ns_send_report(&pkt);
}

#define SUBC_ID_DISABLE_MCU_POLLING (0x25)
#define SUBC_REPORT_DISABLE_MCU_POLLING_LENGTH (3)
void ns_subcommand_disable_mcu_polling(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    //todo
    //ans with ack always for now;
    _ns_subcommand_set_ack(cmd->subcommand_id,DEFAULT_ACK);
    pkt.data.subcommand_report.subcommand_data[0]=0x00;
    pkt.len=SUBC_REPORT_DISABLE_MCU_POLLING_LENGTH;
    ns_send_report(&pkt);
}

#define SUBC_ID_ATTACHMENT_WRITE (0x28)
//#define SUBC_REPORT_ATTACHMENT_WRITE_LENGTH (3)
void ns_subcommand_attachmend_write(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    //todo
    //ans with ack always for now;
    //Does the same job with OUTPUT report 0x12.???
    _ns_subcommand_set_ack(cmd->subcommand_id,DEFAULT_ACK);
    pkt.len=SUBC_REPORT_BASIC_LENGTH;
    ns_send_report(&pkt);
}

#define SUBC_ID_ATTACHMENT_READ (0x29)
#define SUBC_REPORT_ATTACHMENT_READ_LENGTH (36)
void ns_subcommand_attachmend_read(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    //todo
    //ans with ack always for now;
    //where is the data come from??
    _ns_subcommand_set_ack(cmd->subcommand_id,DEFAULT_ACK);
    pkt.len=SUBC_REPORT_ATTACHMENT_READ_LENGTH;
    ns_send_report(&pkt);
}

#define SUBC_ID_SET_ATTACHMENT_STATE (0x2A)
//Replies always with ACK x00 x2A.
//x00 as an ACK here is a bug. Devs forgot to add an ACK reply.
//still this now??? todo : check if its correct
void ns_subcommand_attachment_state(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    uint8_t state=cmd->subcommand_data[0];//gpio pin value
    _ns_subcommand_set_ack(cmd->subcommand_id,0x00);
    pkt.len=SUBC_REPORT_BASIC_LENGTH;
    ns_send_report(&pkt);
}

#define SUBC_ID_GET_ATTACHMENT_INFO (0x2B)
#define SUBC_REPORT_GET_ATTACHMENT_INFO_LENGTH (22)
void ns_subcommand_get_attachment_info(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    //todo : get attachment info. how to get?
    _ns_subcommand_set_ack(cmd->subcommand_id,DEFAULT_ACK);
    pkt.len=SUBC_REPORT_GET_ATTACHMENT_INFO_LENGTH;
    ns_send_report(&pkt);
}


uint8_t indicate_led_status;
#define SUBC_ID_SET_INDICATE_LED (0x30)
void ns_subcommand_set_indicate_led(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    indicate_led_status=cmd->subcommand_data[0];
    //0x00 disable 0x01 enable
    _ns_subcommand_set_ack(cmd->subcommand_id,DEFAULT_ACK);
    pkt.len=SUBC_REPORT_BASIC_LENGTH;
    ns_send_report(&pkt);
    
    //todo write a FSM for pairing sequence
    is_paired=true;
}

#define SUBC_ID_GET_INDICATE_LED (0x31)
#define SUBC_REPORT_GET_INDICATE_LED_LENGTH (3)
void ns_subcommand_get_indicate_led(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    _ns_subcommand_set_ack(cmd->subcommand_id,0xB0);
    pkt.data.subcommand_report.subcommand_data[0]=indicate_led_status;
    pkt.len=SUBC_REPORT_GET_INDICATE_LED_LENGTH;
    ns_send_report(&pkt);
}

#define SUBC_ID_SET_HOME_LED (0x38)
void ns_subcommand_set_home_led(NS_SUBCOMMAND_CB_PARAM){
    //really complecate,do nothing as we dont really have such thing;
    //no reply needed seems;
}


typedef struct{
    uint8_t state;
    struct{
        uint8_t gyro_sensitivity;
        uint8_t acc_sensitivity;
        uint8_t gyro_rate;
        uint8_t acc_rate;
    }imu_sensor_conf;
}_imu_conf;
_imu_conf imu_conf;
#define SUBC_ID_SET_IMU_STATE (0x40)
void ns_subcommand_set_imu_state(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    uint8_t state=cmd->subcommand_data[0];
    imu_conf.state=state;
    //0x00 disable 0x01 enable
    _ns_subcommand_set_ack(cmd->subcommand_id,NS_SUBCOMMAND_STATUS_ACK);
    pkt.len=SUBC_REPORT_BASIC_LENGTH;
    ns_send_report(&pkt);
}

#define SUBC_ID_SET_IMU_CONF (0x41)
#define SUBC_REPORT_SET_IMU_CONF_LENGTH (3)
void ns_subcommand_set_imu_conf(NS_SUBCOMMAND_CB_PARAM){
    if(!imu_conf.state){
        imu_conf.state=0x01;
        //todo : set imu conf to default
        pkt_clr();
        _ns_subcommand_set_ack(cmd->subcommand_id,DEFAULT_ACK);
        pkt.data.subcommand_report.subcommand_data[0]=0x40;
        pkt.data.subcommand_report.subcommand_data[1]=0x01;
        pkt.len=SUBC_REPORT_SET_IMU_CONF_LENGTH;
        ns_send_report(&pkt);
        return;
    }
    memcpy(&imu_conf.imu_sensor_conf,cmd->subcommand_data,sizeof(imu_conf.imu_sensor_conf));
}

#define SUBC_ID_SET_IMU_REGISTER (0x42)
void ns_subcommand_set_imu_register(NS_SUBCOMMAND_CB_PARAM){
    uint8_t addr=cmd->subcommand_data[0];
    uint8_t value=cmd->subcommand_data[2];
}

#define SUBC_ID_READ_IMU_REGISTER (0x43)
#define SUBC_REPORT_READ_IMU_REGISTER_BASIC_LENGTH (4)
void ns_subcommand_read_imu_register(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    uint8_t addr=cmd->subcommand_data[0];
    uint8_t cnt=cmd->subcommand_data[1];
    if(cnt>0x20){
        ESP_LOGE("","%s error read imu reg too many cnt:%d",__func__,cnt);
        cnt=0x20;
    }
    _ns_subcommand_set_ack(cmd->subcommand_id,0xC0);
    pkt.data.subcommand_report.subcommand_data[0]=addr;
    pkt.data.subcommand_report.subcommand_data[1]=cnt;
    //todo read reg
    pkt.len=SUBC_REPORT_READ_IMU_REGISTER_BASIC_LENGTH+cnt;
    ns_send_report(&pkt);
}

uint8_t rumble_state;
#define SUBC_ID_SET_RUMBLE_STATE (0x48)
void ns_subcommand_set_rumble_state(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    rumble_state=cmd->subcommand_data[0];
    //0x00 disable 0x01 enable
    _ns_subcommand_set_ack(cmd->subcommand_id,DEFAULT_ACK);
    pkt.len=SUBC_REPORT_BASIC_LENGTH;
    ns_send_report(&pkt);
}

#define SUBC_ID_GET_BATTERY_VOLTAGE (0x50)
#define SUBC_REPORT_GET_BATTERY_VOLTAGE (4)
void ns_subcommand_get_battery_voltage(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    _ns_subcommand_set_ack(cmd->subcommand_id,0xD0);
    *(uint16_t*)pkt.data.subcommand_report.subcommand_data=0x0618;
    pkt.len=SUBC_REPORT_GET_BATTERY_VOLTAGE;
    ns_send_report(&pkt);
}

#define SUBC_ID_WRITE_CHARGE_SETTING (0x51)
void ns_subcommand_write_charge_setting(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    _ns_subcommand_set_ack(cmd->subcommand_id,DEFAULT_ACK);
    pkt.len=SUBC_REPORT_BASIC_LENGTH;
    ns_send_report(&pkt);
}

#define SUBC_ID_READ_CHARGE_SETTING (0x52)
#define SUBC_REPORT_READ_CHARGE_SETTING_LENGTH (3)
void ns_subcommand_read_charge_setting(NS_SUBCOMMAND_CB_PARAM){
    pkt_clr();
    _ns_subcommand_set_ack(cmd->subcommand_id,0xD1);
    pkt.data.subcommand_report.subcommand_data[0]=0x14;
    pkt.len=SUBC_REPORT_READ_CHARGE_SETTING_LENGTH;
    ns_send_report(&pkt);
}

//todo:more hid subcommand to implement,but doesnt seems to be necessary

void ns_subcommand_callback_init(){
    ns_register_subcommand_cb(SUBC_ID_DEFAULT,ns_subcommand_default);
    ns_register_subcommand_cb(SUBC_ID_GET_DEVICE_INFO,ns_subcommand_get_device_info);
    ns_register_subcommand_cb(SUBC_ID_SET_INPUT_MODE,ns_subcommand_set_input_mode);
    ns_register_subcommand_cb(SUBC_ID_TRIGGER_BUTTONS_ELAPSED_TIME,ns_subcommand_trigger_buttons_elapsed_time);
    ns_register_subcommand_cb(SUBC_ID_GET_PAGE_LIST_STATE,ns_subcommand_get_page_list_state);
    ns_register_subcommand_cb(SUBC_ID_SET_HCI_STATE,ns_subcommand_set_hci_state);
    ns_register_subcommand_cb(SUBC_ID_RESET_PAIRING_INFO,ns_subcommand_reset_pairing_info);
    ns_register_subcommand_cb(SUBC_ID_SET_SHIPPING_MODE,ns_subcommand_set_shipping_mode);
    ns_register_subcommand_cb(SUBC_ID_SPI_READ,ns_subcommand_spi_read);
    ns_register_subcommand_cb(SUBC_ID_SPI_WRITE,ns_subcommand_spi_write);
    ns_register_subcommand_cb(SUBC_ID_SPI_ERASE,ns_subcommand_spi_erase);
    ns_register_subcommand_cb(SUBC_ID_RESET_MCU,ns_subcommand_reset_mcu);
    ns_register_subcommand_cb(SUBC_ID_SET_MCU_CONF,ns_subcommand_set_mcu_conf);
    ns_register_subcommand_cb(SUBC_ID_SET_MCU_STATE,ns_subcommand_set_mcu_state);
    ns_register_subcommand_cb(SUBC_ID_ENABLE_MCU_POLLING,ns_subcommand_enable_mcu_polling);
    ns_register_subcommand_cb(SUBC_ID_DISABLE_MCU_POLLING,ns_subcommand_disable_mcu_polling);
    ns_register_subcommand_cb(SUBC_ID_ATTACHMENT_WRITE,ns_subcommand_attachmend_write);
    ns_register_subcommand_cb(SUBC_ID_ATTACHMENT_READ,ns_subcommand_attachmend_read);
    ns_register_subcommand_cb(SUBC_ID_SET_ATTACHMENT_STATE,ns_subcommand_attachment_state);
    ns_register_subcommand_cb(SUBC_ID_GET_ATTACHMENT_INFO,ns_subcommand_get_attachment_info);
    ns_register_subcommand_cb(SUBC_ID_SET_INDICATE_LED,ns_subcommand_set_indicate_led);
    ns_register_subcommand_cb(SUBC_ID_GET_INDICATE_LED,ns_subcommand_get_indicate_led);
    ns_register_subcommand_cb(SUBC_ID_SET_HOME_LED,ns_subcommand_set_home_led);
    ns_register_subcommand_cb(SUBC_ID_SET_IMU_STATE,ns_subcommand_set_imu_state);
    ns_register_subcommand_cb(SUBC_ID_SET_IMU_CONF,ns_subcommand_set_imu_conf);
    ns_register_subcommand_cb(SUBC_ID_SET_IMU_REGISTER,ns_subcommand_set_imu_register);
    ns_register_subcommand_cb(SUBC_ID_READ_IMU_REGISTER,ns_subcommand_read_imu_register);
    ns_register_subcommand_cb(SUBC_ID_SET_RUMBLE_STATE,ns_subcommand_set_rumble_state);
    ns_register_subcommand_cb(SUBC_ID_GET_BATTERY_VOLTAGE,ns_subcommand_get_battery_voltage);
    ns_register_subcommand_cb(SUBC_ID_WRITE_CHARGE_SETTING,ns_subcommand_write_charge_setting);
    ns_register_subcommand_cb(SUBC_ID_READ_CHARGE_SETTING,ns_subcommand_read_charge_setting);
}