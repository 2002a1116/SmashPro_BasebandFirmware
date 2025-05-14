/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#define SCL_IO_PIN (GPIO_NUM_18)
#define SDA_IO_PIN (GPIO_NUM_5)
#define MASTER_FREQUENCY (400000)
#define IMU_ADDR (0b11010100)
#define PORT_NUMBER -1
#define I2C_TIMEOUT_ONE_BYTE (1)

static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}
uint8_t i2c_write_data(i2c_master_dev_handle_t dev,uint8_t addr,uint8_t* buf,int32_t length){
    if(ESP_OK!=i2c_master_transmit(dev, &addr, 1, I2C_TIMEOUT_ONE_BYTE)){//write addr
        return 1;
    }
    if(ESP_OK!=i2c_master_transmit(dev, buf, length, I2C_TIMEOUT_ONE_BYTE)){//write data
        return 2;
    }
    return 0;
}
uint8_t i2c_read_reg(i2c_master_dev_handle_t dev,uint8_t addr,uint8_t* buf,int32_t length){
    if(ESP_OK!=i2c_master_transmit_receive(dev, &addr, 1, buf, length, I2C_TIMEOUT_ONE_BYTE*length)){//read data
        return 3;
    }
    return 0;
}
void i2c_init(void)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = PORT_NUMBER,
        .scl_io_num = SCL_IO_PIN,
        .sda_io_num = SDA_IO_PIN,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));
        
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_ADDR>>1,
        .scl_speed_hz = MASTER_FREQUENCY,
    };

    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
//i2c_master_probe()
//ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data_wr, DATA_LENGTH, -1));/ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, buf, sizeof(buf), buffer, 2, -1));
}