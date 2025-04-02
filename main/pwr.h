#define DEBUG_NONSLEEP
#define GPIO_WAKE_NUM (GPIO_NUM_19)
#define GPIO_BOOT_NUM (GPIO_NUM_21)
#define GPIO_PIN(x) (1ULL<<(x))
extern uint8_t sleep_flag;
extern uint8_t disable_sleep_flag;
extern TaskHandle_t enter_sleep_task_handle;
extern void enter_sleep_task();
extern void _enter_sleep();
extern void set_disablesleep_bit(uint8_t x,uint8_t v);