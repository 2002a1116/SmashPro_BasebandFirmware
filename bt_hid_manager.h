#include "esp_bt_defs.h"
#define MAX_ADC_CHANNEL_CNT (10)

#define NS_SUBCOMMAND_REPORT_DATA_MAX_LENGTH (34)
#define NS_MCU_FW_UPDATE_REPORT_DATA_MAX_LENGTH (35)
#define NS_BASIC_REPORT_RESERVED_MAX_LENGTH (35)
#define NS_SUBCOMMAND_CMD_DATA_MAX_LENGTH (38)
#define NS_MCU_FW_UPDATE_CMD_DATA_MAX_LENGTH (308)
#define NS_MCU_CALL_CMD_DATA_MAX_LENGTH (38)
#define NS_ATTACHMENT_CMD_DATA_MAX_LENGTH (38)
#define NS_PACKET_TYPE_MAX_VALUE (128)
#define NS_SUBCOMMAND_ID_MAX_VLAUE (256)
#define NS_STD_REPORT_BASIC_LENGTH (13)
#define NS_SUBCOMMAND_CMD_HEADER_LENGTH (11)

#define NS_SUBCOMMAND_STATUS_ACK (0x80)
#define NS_SUBCOMMAND_STATUS_NACK (0x00)

typedef enum{
    NS_REPORT_MODE_STD,
    NS_REPORT_MODE_NFC_IR,
    NS_REPORT_MODE_SIMPLE_HID
}NS_REPORT_MODE;

#pragma pack(push,1)
typedef struct{
    union{
        struct{
            uint8_t subcommand_status;//ack or nack
            uint8_t subcommand_id;
            uint8_t subcommand_data[NS_SUBCOMMAND_REPORT_DATA_MAX_LENGTH];
        }subcommand_report;
        uint8_t mcu_fw_update_report[NS_MCU_FW_UPDATE_REPORT_DATA_MAX_LENGTH];
        uint8_t raw_report_reserved[NS_BASIC_REPORT_RESERVED_MAX_LENGTH];
        struct
        {
           uint32_t acc_sample0:24;
           uint32_t gyro_sample0:24;
           uint32_t acc_sample1:24;
           uint32_t gyro_sample1:24;
           uint32_t acc_sample2:24;
           uint32_t gyro_sample2:24;
        }imu_report;
    };
}std_report_data;
typedef struct{
    uint32_t button_status:24;
    uint32_t ljoy_status:24;
    uint32_t rjoy_status:24;
}peripheral_data;
typedef struct{
    uint8_t typ;
    uint16_t button_status;
    uint8_t stick_hat_data;
    uint16_t ljoy_status_hori;
    uint16_t ljoy_status_vert;
    uint16_t rjoy_status_hori;
    uint16_t rjoy_status_vert;
}simple_report;
typedef struct{
    uint8_t typ;
    uint8_t timer;
    uint8_t con_info:4;
    uint8_t battery_status:4;
    peripheral_data input_data; 
    uint8_t rumble_status;
    std_report_data data;
}std_report;
typedef struct{
    uint8_t typ;
    uint8_t timer;
    uint32_t rumble_data_left;
    uint32_t rumble_data_right;
}cmd_std;
typedef struct{
    cmd_std cmd_header;
    uint8_t subcommand_id;
    uint8_t subcommand_data[NS_SUBCOMMAND_CMD_DATA_MAX_LENGTH];
}cmd_subcommand;
typedef struct{
    cmd_std cmd_header;
    uint8_t mcu_fw_update_data[NS_MCU_FW_UPDATE_CMD_DATA_MAX_LENGTH];
}cmd_mcu_fw_update;
typedef struct{
    cmd_std cmd_header;
    uint8_t mcu_call_data[NS_MCU_CALL_CMD_DATA_MAX_LENGTH];
}cmd_mcu_call;
typedef struct{
    cmd_std cmd_header;
    uint8_t attachment_data[NS_ATTACHMENT_CMD_DATA_MAX_LENGTH];
}cmd_attachment;
#pragma pack(pop)

typedef struct{
    std_report_data data;
    void* oob_data;
    uint16_t len;
    uint32_t oob_len;
}report_packet;
typedef struct{
    cmd_std* cmd;
    uint8_t len;
}cmd_packet;

extern NS_REPORT_MODE report_mode;
extern uint8_t bt_addr[ESP_BD_ADDR_LEN];
extern uint8_t con_addr[ESP_BD_ADDR_LEN];
extern uint8_t is_connected,is_paired,auto_con;
extern uint32_t average_packet_gap,last_packet_send;


esp_err_t ns_bt_hid_send_raw(void*,int);
esp_err_t ns_send_interrupt(simple_report*);
int ns_send_report(report_packet*);
void ns_register_subcommand_cb(int,void (*)(cmd_subcommand*,uint8_t));//SET CB NULL TO UNREG
void ns_set_peripheral_data_getter(void (*)(peripheral_data*));
void bt_hid_init();