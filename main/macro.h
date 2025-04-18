#pragma once
#include "soc/gpio_num.h"
#include "hal/adc_types.h"

#define GPIO_BUTTON_LS GPIO_NUM_33
#define GPIO_BUTTON_RS GPIO_NUM_17
#define GPIO_BUTTON_X GPIO_NUM_18
#define GPIO_BUTTON_Y GPIO_NUM_21
#define GPIO_BUTTON_A GPIO_NUM_5
#define GPIO_BUTTON_B GPIO_NUM_16
#define GPIO_BUTTON_UP GPIO_NUM_12
#define GPIO_BUTTON_DOWN GPIO_NUM_27
#define GPIO_BUTTON_LEFT GPIO_NUM_26
#define GPIO_BUTTON_RIGHT GPIO_NUM_14
#define GPIO_BUTTON_L GPIO_NUM_4
#define GPIO_BUTTON_R GPIO_NUM_19
#define GPIO_BUTTON_ZL GPIO_NUM_25
#define GPIO_BUTTON_ZR GPIO_NUM_22
#define GPIO_BUTTON_MINUS GPIO_NUM_13
#define GPIO_BUTTON_PLUS GPIO_NUM_4
#define GPIO_BUTTON_HOME GPIO_NUM_15
//#define GPIO_BUTTON_CAP GPIO_NUM_16

#define BT_BUTTON_SL_L (5)
#define BT_BUTTON_SR_L (4)
#define BT_BUTTON_SL_R (20)
#define BT_BUTTON_SR_R (21)
#define BT_BUTTON_X (1)
#define BT_BUTTON_Y (0)
#define BT_BUTTON_A (3)
#define BT_BUTTON_B (2)
#define BT_BUTTON_UP (17)
#define BT_BUTTON_DOWN (16)
#define BT_BUTTON_LEFT (19)
#define BT_BUTTON_RIGHT (18)
#define BT_BUTTON_L (22)
#define BT_BUTTON_R (6)
#define BT_BUTTON_ZL (23)
#define BT_BUTTON_ZR (7)
#define BT_BUTTON_MINUS (8)
#define BT_BUTTON_PLUS (9)
#define BT_BUTTON_HOME (12)
#define BT_BUTTON_CAP (13)
#define BT_BUTTON_RS (10)
#define BT_BUTTON_LS (11)

#define BT_BUTTON_SIMPLE_B (0)
#define BT_BUTTON_SIMPLE_A (1)
#define BT_BUTTON_SIMPLE_Y (2)
#define BT_BUTTON_SIMPLE_X (3)
#define BT_BUTTON_SIMPLE_L (4)
#define BT_BUTTON_SIMPLE_R (5)
#define BT_BUTTON_SIMPLE_ZL (6)
#define BT_BUTTON_SIMPLE_ZR (7)
#define BT_BUTTON_SIMPLE_MINUS (8)
#define BT_BUTTON_SIMPLE_PLUS (9)
#define BT_BUTTON_SIMPLE_LS (10)
#define BT_BUTTON_SIMPLE_RS (11)
#define BT_BUTTON_SIMPLE_HOME (12)
#define BT_BUTTON_SIMPLE_CAP (13)

#define ADC_CHANNEL_LJOYS_HORI ADC_CHANNEL_3
#define ADC_CHANNEL_LJOYS_VERT ADC_CHANNEL_0
#define ADC_CHANNEL_RJOYS_HORI ADC_CHANNEL_6
#define ADC_CHANNEL_RJOYS_VERT ADC_CHANNEL_4
#define ADC_CHANNEL_BAT_VOLT ADC_CHANNEL_7

#define GPIO_NUM_MAX_VALUE (GPIO_NUM_33)
#define BT_BUTTON_MAX_CNT (24)

#define BT_NUM_TO_BIT(x) (1<<(x))
#define BT_VALUE_TO_BIT(v,x) ((v)<<(x))