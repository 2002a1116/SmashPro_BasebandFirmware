#pragma once
#include <esp_err.h>
esp_err_t nvs_set(char* name,void* data,size_t len);
esp_err_t nvs_get(char* name,void* data,size_t len);
esp_err_t nvs_init();