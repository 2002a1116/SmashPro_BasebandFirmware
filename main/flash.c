
#include "spi_flash_mmap.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
esp_err_t nvs_set(char* name,void* data,size_t len)
{
    nvs_handle handle;
    esp_err_t err;
    if(!name)return ESP_ERR_NVS_CORRUPT_KEY_PART;
    do{
        err = nvs_open("storage", NVS_READWRITE, &handle);
        if (err != ESP_OK)break;
        err = nvs_set_blob(handle, name, data, len);
        if(err!=ESP_OK)break;
        err = nvs_commit(handle);
    }while(false);
    nvs_close(handle);
    return err;
}
esp_err_t nvs_get_len(char* name,size_t *len)
{
    nvs_handle handle;
    esp_err_t err;
    if(!name)return ESP_ERR_NVS_CORRUPT_KEY_PART;
    do{
        err = nvs_open("storage", NVS_READWRITE, &handle);
        if (err != ESP_OK)break;
        err = nvs_get_blob(handle, name, NULL, len);
    }while(false);
    nvs_close(handle);
    return err;
}
esp_err_t nvs_get(char* name,void* data,size_t len)
{
    nvs_handle handle;
    esp_err_t err;
    size_t t;
    if(!name)return ESP_ERR_NVS_CORRUPT_KEY_PART;
    do{
        err = nvs_open("storage", NVS_READWRITE, &handle);
        if (err != ESP_OK)break;
        err = nvs_get_blob(handle, name, NULL, &t);
        if (err != ESP_OK || t<len) break;
        err = nvs_get_blob(handle, name, data, &len);
    }while(false);
    nvs_close(handle);
    return err;
}
esp_err_t nvs_init(){
    esp_err_t ret = nvs_flash_init();
    ESP_LOGW("","nvs init res %d",ret);
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    return ret;
}