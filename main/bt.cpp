/*#include "spi_flash_mmap.h"

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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/periph_ctrl.h"

#include "flash.h"

typedef char byte;
typedef struct _input_data{
    uint32_t button;
    uint32_t ljoys;
    uint32_t rjoys;
    uint32_t timeout_at; 
}input_data;
class bt_hid_instance{
public:
    byte[ESP_BD_ADDR_LEN] host_mac;
    byte[ESP_BD_ADDR_LEN] oppo_mac;
    byte connected,paired;
    SemaphoreHandle_t xSemaphore;
}
*/