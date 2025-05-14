/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "macro.h"
#include "hardware.h"

/**
 * Brief:
 * This test code shows how to configure gpio and how to use gpio interrupt.
 *
 * GPIO status:
 * GPIO18: output (ESP32C2/ESP32H2 uses GPIO8 as the second output pin)
 * GPIO19: output (ESP32C2/ESP32H2 uses GPIO9 as the second output pin)
 * GPIO4:  input, pulled up, interrupt from rising edge and falling edge
 * GPIO5:  input, pulled up, interrupt from rising edge.
 *
 * Note. These are the default GPIO pins to be used in the example. You can
 * change IO pins in menuconfig.
 *
 * Test:
 * Connect GPIO18(8) with GPIO4
 * Connect GPIO19(9) with GPIO5
 * Generate pulses on GPIO18(8)/19(9), that triggers interrupt on GPIO4/5
 *
 */

/*
 * Let's say, GPIO_OUTPUT_IO_0=18, GPIO_OUTPUT_IO_1=19
 * In binary representation,
 * 1ULL<<GPIO_OUTPUT_IO_0 is equal to 0000000000000000000001000000000000000000 and
 * 1ULL<<GPIO_OUTPUT_IO_1 is equal to 0000000000000000000010000000000000000000
 * GPIO_OUTPUT_PIN_SEL                0000000000000000000011000000000000000000
 * */
#define GPIO_INPUT_IO_0     4
#define GPIO_INPUT_IO_1     5
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
/*
 * Let's say, GPIO_INPUT_IO_0=4, GPIO_INPUT_IO_1=5
 * In binary representation,
 * 1ULL<<GPIO_INPUT_IO_0 is equal to 0000000000000000000000000000000000010000 and
 * 1ULL<<GPIO_INPUT_IO_1 is equal to 0000000000000000000000000000000000100000
 * GPIO_INPUT_PIN_SEL                0000000000000000000000000000000000110000
 * */
#define ESP_INTR_FLAG_DEFAULT 0


static QueueHandle_t gpio_evt_queue = NULL;
static void (*_gpio_isr_cb)(uint8_t,uint8_t);
static void (*_gpio_upd_cb)();
static uint8_t gpio_num_to_bt_num_tb[40];
static uint8_t gpio_num_to_bt_num_simple_tb[40];
static uint8_t bt_num_to_gpio_num_tb[BT_BUTTON_MAX_CNT];
static uint64_t gpio_debounce_start_tick[GPIO_NUM_MAX];
static uint8_t gpio_debounce_value[GPIO_NUM_MAX];
static uint64_t gpio_debounce_tick=10000;
void gpio_set_debounce_time(uint8_t t)
{
    gpio_debounce_tick=t*1000;
}
void gpio_trans_init()
{
    memset(gpio_num_to_bt_num_tb,-1,sizeof(gpio_num_to_bt_num_tb));
    memset(gpio_debounce_start_tick,-1,sizeof(gpio_debounce_start_tick));
    gpio_num_to_bt_num_tb[GPIO_BUTTON_LS]=BT_BUTTON_LS;
    gpio_num_to_bt_num_tb[GPIO_BUTTON_RS]=BT_BUTTON_RS;
    gpio_num_to_bt_num_tb[GPIO_BUTTON_X]=BT_BUTTON_X;
    gpio_num_to_bt_num_tb[GPIO_BUTTON_Y]=BT_BUTTON_Y;
    gpio_num_to_bt_num_tb[GPIO_BUTTON_A]=BT_BUTTON_A;
    gpio_num_to_bt_num_tb[GPIO_BUTTON_B]=BT_BUTTON_B;
    gpio_num_to_bt_num_tb[GPIO_BUTTON_UP]=BT_BUTTON_UP;
    gpio_num_to_bt_num_tb[GPIO_BUTTON_DOWN]=BT_BUTTON_DOWN;
    gpio_num_to_bt_num_tb[GPIO_BUTTON_LEFT]=BT_BUTTON_LEFT;
    gpio_num_to_bt_num_tb[GPIO_BUTTON_RIGHT]=BT_BUTTON_RIGHT;
    gpio_num_to_bt_num_tb[GPIO_BUTTON_L]=BT_BUTTON_L;
    gpio_num_to_bt_num_tb[GPIO_BUTTON_R]=BT_BUTTON_R;
    gpio_num_to_bt_num_tb[GPIO_BUTTON_ZL]=BT_BUTTON_ZL;
    gpio_num_to_bt_num_tb[GPIO_BUTTON_ZR]=BT_BUTTON_ZR;
    gpio_num_to_bt_num_tb[GPIO_BUTTON_MINUS]=BT_BUTTON_MINUS;
    gpio_num_to_bt_num_tb[GPIO_BUTTON_PLUS]=BT_BUTTON_PLUS;
    gpio_num_to_bt_num_tb[GPIO_BUTTON_HOME]=BT_BUTTON_HOME;
    //gpio_num_to_bt_num_tb[GPIO_BUTTON_CAP]=BT_BUTTON_CAP;

    bt_num_to_gpio_num_tb[BT_BUTTON_LS]=GPIO_BUTTON_LS;
    bt_num_to_gpio_num_tb[BT_BUTTON_RS]=GPIO_BUTTON_RS;
    bt_num_to_gpio_num_tb[BT_BUTTON_X]=GPIO_BUTTON_X;
    bt_num_to_gpio_num_tb[BT_BUTTON_Y]=GPIO_BUTTON_Y;
    bt_num_to_gpio_num_tb[BT_BUTTON_A]=GPIO_BUTTON_A;
    bt_num_to_gpio_num_tb[BT_BUTTON_B]=GPIO_BUTTON_B;
    bt_num_to_gpio_num_tb[BT_BUTTON_UP]=GPIO_BUTTON_UP;
    bt_num_to_gpio_num_tb[BT_BUTTON_DOWN]=GPIO_BUTTON_DOWN;
    bt_num_to_gpio_num_tb[BT_BUTTON_LEFT]=GPIO_BUTTON_LEFT;
    bt_num_to_gpio_num_tb[BT_BUTTON_RIGHT]=GPIO_BUTTON_RIGHT;
    bt_num_to_gpio_num_tb[BT_BUTTON_L]=GPIO_BUTTON_L;
    bt_num_to_gpio_num_tb[BT_BUTTON_R]=GPIO_BUTTON_R;
    bt_num_to_gpio_num_tb[BT_BUTTON_ZL]=GPIO_BUTTON_ZL;
    bt_num_to_gpio_num_tb[BT_BUTTON_ZR]=GPIO_BUTTON_ZR;
    bt_num_to_gpio_num_tb[BT_BUTTON_MINUS]=GPIO_BUTTON_MINUS;
    bt_num_to_gpio_num_tb[BT_BUTTON_PLUS]=GPIO_BUTTON_PLUS;
    bt_num_to_gpio_num_tb[BT_BUTTON_HOME]=GPIO_BUTTON_HOME;

    gpio_num_to_bt_num_simple_tb[GPIO_BUTTON_A]=BT_BUTTON_SIMPLE_A;
    gpio_num_to_bt_num_simple_tb[GPIO_BUTTON_B]=BT_BUTTON_SIMPLE_B;
    gpio_num_to_bt_num_simple_tb[GPIO_BUTTON_X]=BT_BUTTON_SIMPLE_X;
    gpio_num_to_bt_num_simple_tb[GPIO_BUTTON_Y]=BT_BUTTON_SIMPLE_Y;
    gpio_num_to_bt_num_simple_tb[GPIO_BUTTON_L]=BT_BUTTON_SIMPLE_L;
    gpio_num_to_bt_num_simple_tb[GPIO_BUTTON_R]=BT_BUTTON_SIMPLE_R;
    gpio_num_to_bt_num_simple_tb[GPIO_BUTTON_ZL]=BT_BUTTON_SIMPLE_ZL;
    gpio_num_to_bt_num_simple_tb[GPIO_BUTTON_ZR]=BT_BUTTON_SIMPLE_ZR;
    gpio_num_to_bt_num_simple_tb[GPIO_BUTTON_MINUS]=BT_BUTTON_SIMPLE_MINUS;
    gpio_num_to_bt_num_simple_tb[GPIO_BUTTON_PLUS]=BT_BUTTON_SIMPLE_PLUS;
    gpio_num_to_bt_num_simple_tb[GPIO_BUTTON_LS]=BT_BUTTON_SIMPLE_LS;
    gpio_num_to_bt_num_simple_tb[GPIO_BUTTON_RS]=BT_BUTTON_SIMPLE_RS;
    gpio_num_to_bt_num_simple_tb[GPIO_BUTTON_HOME]=BT_BUTTON_SIMPLE_HOME;
    //gpio_num_to_bt_num_simple_tb[GPIO_BUTTON_CAP]=BT_BUTTON_SIMPLE_CAP;
    //bt_num_to_gpio_num_tb[BT_BUTTON_CAP]=GPIO_BUTTON_CAP;
}
inline uint8_t gpio_num_to_bt_num(uint32_t gpio_num){
    return gpio_num_to_bt_num_tb[gpio_num];
}
inline uint8_t bt_num_to_gpio_num(uint8_t bt_num){
    return bt_num_to_gpio_num_tb[bt_num];
}
inline uint8_t gpio_num_to_bt_num_simple(uint32_t gpio_num){
    return gpio_num_to_bt_num_simple_tb[gpio_num];
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    //uint8_t io_num=(uint32_t)arg;
    if(gpio_debounce_start_tick[(uint32_t)arg]==GPIO_DEBOUNCE_INVALID_TICK)
        gpio_debounce_start_tick[(uint32_t)arg]=esp_timer_get_time();
    //gpio_debounce_value[(uint32_t)arg]=gpio_get_level((uint32_t)arg);
    /*bool r=c;
    if(r)gpio_num|=GPIO_VALUE_BIT;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);*/
}

static void gpio_task_example(void* arg)
{
    uint64_t t;
    uint8_t upd=false,v;
    for (;;) {
        /*if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%"PRIu32"] intr\n", io_num);
            if(_gpio_isr_cb)
                _gpio_isr_cb(io_num);
        }*/
        //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        t=esp_timer_get_time()+gpio_debounce_tick;
        for(int i=0;i<=GPIO_NUM_MAX_VALUE;++i){
            if(gpio_debounce_start_tick[i]<=t)
            {
                upd=true;
                printf("io_upd %d\n",i);
                if(_gpio_isr_cb)
                    _gpio_isr_cb(i,gpio_get_level(i));
                    //_gpio_isr_cb(i|(gpio_get_level(bt_num_to_gpio_num(i))?(GPIO_VALUE_BIT):0));
                gpio_debounce_start_tick[i]=GPIO_DEBOUNCE_INVALID_TICK;
                t=esp_timer_get_time()+gpio_debounce_tick;
            }
        }
        if(upd&&_gpio_upd_cb)_gpio_upd_cb(),upd=false;
        vTaskDelay(1/portTICK_PERIOD_MS);
    }
}
void gpio_init(void (*gpio_isr_cb)(uint8_t,uint8_t),void (*gpio_upd_cb)())
{
    _gpio_isr_cb=gpio_isr_cb;
    _gpio_upd_cb=gpio_upd_cb;
    gpio_trans_init();
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};

    //interrupt of rising edge
    //io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_PIN(GPIO_BUTTON_LS)|
                            GPIO_PIN(GPIO_BUTTON_RS)|
                            GPIO_PIN(GPIO_BUTTON_X)|
                            GPIO_PIN(GPIO_BUTTON_Y)|
                            GPIO_PIN(GPIO_BUTTON_A)|
                            GPIO_PIN(GPIO_BUTTON_B)|
                            GPIO_PIN(GPIO_BUTTON_DOWN)|
                            GPIO_PIN(GPIO_BUTTON_UP)|
                            GPIO_PIN(GPIO_BUTTON_LEFT)|
                            GPIO_PIN(GPIO_BUTTON_RIGHT)|
                            GPIO_PIN(GPIO_BUTTON_L)|
                            GPIO_PIN(GPIO_BUTTON_R)|
                            GPIO_PIN(GPIO_BUTTON_ZL)|
                            GPIO_PIN(GPIO_BUTTON_ZR)|
                            GPIO_PIN(GPIO_BUTTON_MINUS)|
                            GPIO_PIN(GPIO_BUTTON_PLUS)|
                            GPIO_PIN(GPIO_BUTTON_HOME)
                            //GPIO_PIN(GPIO_BUTTON_CAP)
    ;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-down mode
    //io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(50, sizeof(uint32_t));
    return;
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
 
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_BUTTON_LS, gpio_isr_handler, (void*) GPIO_BUTTON_LS);
    gpio_isr_handler_add(GPIO_BUTTON_RS, gpio_isr_handler, (void*) GPIO_BUTTON_RS);
    gpio_isr_handler_add(GPIO_BUTTON_X, gpio_isr_handler, (void*) GPIO_BUTTON_X);
    gpio_isr_handler_add(GPIO_BUTTON_Y, gpio_isr_handler, (void*) GPIO_BUTTON_Y);
    gpio_isr_handler_add(GPIO_BUTTON_A, gpio_isr_handler, (void*) GPIO_BUTTON_A);
    gpio_isr_handler_add(GPIO_BUTTON_B, gpio_isr_handler, (void*) GPIO_BUTTON_B);
    gpio_isr_handler_add(GPIO_BUTTON_DOWN, gpio_isr_handler, (void*) GPIO_BUTTON_DOWN);
    gpio_isr_handler_add(GPIO_BUTTON_UP, gpio_isr_handler, (void*) GPIO_BUTTON_UP);
    gpio_isr_handler_add(GPIO_BUTTON_LEFT, gpio_isr_handler, (void*) GPIO_BUTTON_LEFT);
    gpio_isr_handler_add(GPIO_BUTTON_RIGHT, gpio_isr_handler, (void*) GPIO_BUTTON_RIGHT);
    gpio_isr_handler_add(GPIO_BUTTON_L, gpio_isr_handler, (void*) GPIO_BUTTON_L);
    gpio_isr_handler_add(GPIO_BUTTON_R, gpio_isr_handler, (void*) GPIO_BUTTON_R);
    gpio_isr_handler_add(GPIO_BUTTON_ZL, gpio_isr_handler, (void*) GPIO_BUTTON_ZL);
    gpio_isr_handler_add(GPIO_BUTTON_ZR, gpio_isr_handler, (void*) GPIO_BUTTON_ZR);
    gpio_isr_handler_add(GPIO_BUTTON_MINUS, gpio_isr_handler, (void*) GPIO_BUTTON_MINUS);
    gpio_isr_handler_add(GPIO_BUTTON_PLUS, gpio_isr_handler, (void*) GPIO_BUTTON_PLUS);
    gpio_isr_handler_add(GPIO_BUTTON_HOME, gpio_isr_handler, (void*) GPIO_BUTTON_HOME);
    //gpio_isr_handler_add(GPIO_BUTTON_CAP, gpio_isr_handler, (void*) GPIO_BUTTON_CAP);
}