#pragma once
#define GPIO_DEBOUNCE_INVALID_TICK (-1)

#define GPIO_PIN(x) (1ULL<<(x))
#define GPIO_VALUE_BIT (1<<31)

void gpio_init(void (*gpio_isr_cb)(uint8_t,uint8_t),void(*gpio_upd_cb)()); 
uint8_t gpio_num_to_bt_num(uint32_t);
uint8_t gpio_num_to_bt_num_simple(uint32_t);
uint8_t bt_num_to_gpio_num(uint8_t);
void gpio_set_debounce_time(uint8_t t);