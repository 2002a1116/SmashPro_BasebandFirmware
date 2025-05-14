           /*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_filter.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "ananlog.h"
#include "macro.h"

#define EXAMPLE_ADC_UNIT ADC_UNIT_1
#define _EXAMPLE_ADC_UNIT_STR(unit) #unit
#define EXAMPLE_ADC_UNIT_STR(unit) _EXAMPLE_ADC_UNIT_STR(unit)
#define EXAMPLE_ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_12
#define EXAMPLE_ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define EXAMPLE_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_GET_CHANNEL(p_data) ((p_data)->type1.channel)
#define EXAMPLE_ADC_GET_DATA(p_data) ((p_data)->type1.data)
#else
#define EXAMPLE_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EXAMPLE_ADC_GET_CHANNEL(p_data) ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data) ((p_data)->type2.data)
#endif

#define EXAMPLE_READ_LEN 256

static adc_channel_t channel[5] = {ADC_CHANNEL_LJOYS_HORI, ADC_CHANNEL_LJOYS_VERT, ADC_CHANNEL_RJOYS_HORI, ADC_CHANNEL_RJOYS_VERT,ADC_CHANNEL_BAT_VOLT};

static TaskHandle_t s_task_handle;
static const char *TAG = "EXAMPLE";

adc_continuous_handle_t handle = NULL;
adc_cali_handle_t cali_handle = NULL;

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    // Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = EXAMPLE_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20 * 1000,
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++)
    {
        adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
        adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}
void ananlog_read(uint32_t *res, uint8_t len)
{
    uint8_t result[EXAMPLE_READ_LEN] = {0};
    memset(result, 0xcc, EXAMPLE_READ_LEN);
    uint32_t ret_num = 0;
    esp_err_t ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
    int volt = 0;
    if (ret == ESP_OK)
    {
        // ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
        {
            adc_digi_output_data_t *p = (void *)&result[i];
            uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
            uint32_t data = EXAMPLE_ADC_GET_DATA(p);
            adc_cali_raw_to_voltage(cali_handle, data, &volt);
            /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
            if (chan_num < SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT))
            {
                if (chan_num < len && (res[chan_num] == -1 || abs((int)res[chan_num] - (int)data) > 32))
                {
                    //ESP_LOGI(TAG, "Channel: %" PRIu32 ", Value: %u,valt: %d", chan_num, (unsigned int)data, volt);
                    res[chan_num] = abs(data-2048)>50?data:2048;
                }
                // ESP_LOGI(TAG, "Unit: %s, Channel: %"PRIu32", Value: %"PRIx32, unit, chan_num, data);
            }
            else
            {
                ESP_LOGW(TAG, "Invalid data [chan %" PRIu32 "_%" PRIx32 "]", chan_num, data);
            }
        }
        /**
         * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
         * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
         * usually you don't need this delay (as this task will block for a while).
         */
        //vTaskDelay(1/portTICK_PERIOD_MS);
    }
    else if (ret == ESP_ERR_TIMEOUT)
    {
        // We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
        ;
    }
}
void ananlog_init(void)
{
    esp_err_t ret;

    s_task_handle = xTaskGetCurrentTaskHandle();

    // adc_continuous_iir_filter_config_t filter;
    // adc_iir_filter_handle_t iir_handle = NULL;
    // filter.unit=ADC_UNIT_1;
    // filter.channel=ADC_CHANNEL_2;
    // filter.coeff=ADC_DIGI_IIR_FILTER_COEFF_16;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);
    ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = EXAMPLE_ADC_UNIT,
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = EXAMPLE_ADC_BIT_WIDTH};
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle));
    // adc_new_continuous_iir_filter(handle,&filter,&iir_handle);
    // adc_continuous_iir_filter_enable(iir_handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    //ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));
}
static void ananlog_clear()
{
    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}