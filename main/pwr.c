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

uint8_t disable_sleep_flag=0;
TaskHandle_t enter_sleep_task_handle=NULL;
TickType_t last_sleep_tick=0;
static uart_packet reply;
void set_nosleep_bit(uint8_t x,uint8_t state)
{
    if(state)
        disable_sleep_flag|=x;
    else
        disable_sleep_flag&=~(x);
}
void enter_sleep_task()
{
    last_sleep_tick=xTaskGetTickCount();
    while(1)
    {
        vTaskDelay(1000/portTICK_PERIOD_MS);
        if(xTaskGetTickCount()-last_sleep_tick<MINIUM_WAKE_TICKS)
            continue;
        ESP_LOGI(__func__,"DISABLE:%d",disable_sleep_flag);
        if(disable_sleep_flag)
            continue;
        reply.typ=UART_PKG_PWR_CONTROL;
        reply.id=0x1;
        send_uart_pkt(&reply);
        _enter_sleep();
        last_sleep_tick=xTaskGetTickCount();
    }
}
void _enter_sleep(){
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = GPIO_PIN(GPIO_BOOT_NUM);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_down_en = 1;
        gpio_config(&io_conf);
        ESP_LOGW(__func__," start");
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
        esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_ON);
        uart_set_wakeup_threshold(INTERNAL_UART_PORT_NUM,5);
        esp_sleep_enable_uart_wakeup(INTERNAL_UART_PORT_NUM);
        esp_sleep_enable_bt_wakeup();
        //esp_sleep_enable_ext0_wakeup(GPIO_WAKE_NUM,0);
        //esp_sleep_enable_ext0_wakeup(GPIO_BOOT_NUM,1);
        //gpio_wakeup_enable(GPIO_WAKE_NUM,0);
        gpio_wakeup_enable(GPIO_BOOT_NUM,GPIO_INTR_HIGH_LEVEL);
        esp_sleep_enable_gpio_wakeup();
        esp_light_sleep_start();
        last_sleep_tick=0;
        ESP_LOGW(__func__," quit");
}