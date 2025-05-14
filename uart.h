#pragma once
#include "driver/gpio.h"
#include "bt_hid_manager.h"
#include "esp_attr.h"
#define INTERNAL_UART_PKG_HEADER_MASK (0x80)
#define INTERNAL_UART_PKG_LOAD_MASK (~(INTERNAL_UART_PKG_HEADER_MASK|(INTERNAL_UART_PKG_HEADER_MASK<<8)|(INTERNAL_UART_PKG_HEADER_MASK<<16)))
#define INTERNAL_UART_BAUD_RATE (3000000)
//#define INTERNAL_UART_BAUD_RATE (921600)
#define INTERNAL_UART_GAP (1)
#define INTERNAL_UART_INTR_ALLOC_FLAGS ESP_INTR_FLAG_IRAM
//#define INTERNAL_UART_PORT_NUM (2)
#define INTERNAL_UART_PORT_NUM (1)
#define INTERNAL_UART_BUF_SIZE (1024)
#define INTERNAL_UART_TXD_GPIO_NUM (GPIO_NUM_17)
#define INTERNAL_UART_RXD_GPIO_NUM (GPIO_NUM_16)
#define INTERNAL_UART_PKG_SIZE (13)
#define INTERNAL_UART_PKG_LOAD_LENGTH (9)
//#define INTERNAL_UART_RST_GPIO_NUM (GPIO_NUM_18)
#pragma pack(push,1)
typedef struct __uart_packet{
    struct{
        uint8_t id:7;
        uint8_t starter:1;
    };
    union{
        struct{
            uint8_t typ:5;
            uint8_t hb2:2;
            uint8_t _empty:1;
            uint8_t hb1:7;
            uint8_t _empty2:1;
            uint8_t load[9];
        };
        uint8_t data[11];
    };
    struct{
        uint8_t cksum:7;
        uint8_t _empty3:1;
    };
}uart_packet,*puart_packet;
enum UART_PACKET_TYP{
    UART_PKG_INPUT_DATA=0x01,
    UART_PKG_INPUT_REQ=0x02,
    UART_PKG_CONNECT_CONTROL=0x03,
    UART_PKG_RUMBLE_FRAME=0x04,
    UART_PKG_CONTROL_DATA=0x05,
    UART_PKG_PWR_CONTROL=0x06,
    UART_PKG_TEST_ECHO=0x07,
    UART_PKG_CH32_FLASH_READ=0x08,
    UART_PKG_CH32_FLASH_WRITE=0x09
};
extern void uart_init();
extern void IRAM_ATTR uart_get_peripheral_data(peripheral_data* data);
extern void IRAM_ATTR uart_pkt_handler(puart_packet pkg);
extern void IRAM_ATTR uart_recv_task();
extern void IRAM_ATTR uart_get_peripheral_data(peripheral_data* data);
extern void uart_send(puart_packet pkg);
extern void encode_uart_pkt(uart_packet* pkt);
extern void decode_uart_pkt(uart_packet* pkt);
extern uint8_t check_uart_pkt(uart_packet* pkt);
extern uint8_t send_uart_pkt(uart_packet* pkt);
extern uint8_t send_uart_large(uint8_t* buf,uint8_t len,uint8_t typ);