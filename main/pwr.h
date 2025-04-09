//#define DEBUG_NONSLEEP
#define GPIO_WAKE_NUM (GPIO_NUM_19)
#define GPIO_BOOT_NUM (GPIO_NUM_21)
#define GPIO_PIN(x) (1ULL<<(x))

#define MINIUM_WAKE_MS (4*1000)
#define MINIUM_WAKE_TICKS (MINIUM_WAKE_MS*portTICK_PERIOD_MS)

enum{
    NOSLEEP_USB = 0x01,
    NOSLEEP_BT_CONNECTING = 0x02,
    NOSLEEP_BT_CONNECTED = 0x04,
    NOSLEEP_BT_LISTENING = 0x08,
    NOSLEEP_SET_STATUS = 0x10,
    NOSLEEP_FORCED = 0x40,
    NOSLEEP_DEBUG = 0x80,
};

extern uint8_t disable_sleep_flag;
extern TaskHandle_t enter_sleep_task_handle;
extern void enter_sleep_task();
extern void _enter_sleep();
extern void set_nosleep_bit(uint8_t x,uint8_t state);