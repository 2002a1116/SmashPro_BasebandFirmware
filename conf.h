/*
 * conf.h
 *
 *  Created on: 2024年11月26日
 *      Author: Reed
 */

#ifndef USER_CONF_H_
#define USER_CONF_H_
#include <stdint.h>

#define FW_VERSION_HIGH (0)
#define FW_VERSION_LOW (3)

#define NS_SPI_USER_JOYSTICK_CALIBRATION_ADDR (0x8010)
#define NS_SPI_USER_IMU_CALIBRATION_ADDR (0X8026)

#define RGB_MAX_CNT (29)

/*
Firmware
The firmware is stored inside the flash in Broadcom's PatchRAM format as follows:

Offset  Size    Description
0x0 0x1000  StaticSection
0x1000  0x1000  FailsafeSection
0x2000  0x1000  VolatileSection
0x3000  0x1000  VolatileSectionBackup1
0x4000  0x1000  VolatileSectionBackup2
0x5000  0x1000  ShipmentInfo
0x6000  0x1000  FactoryConfiguration
0x7000  0x1000  Reserved
0x8000  0x1000  UserCalibration
0x9000  0x7000  Reserved
0x10000 0x18000 DynamicSection1
0x28000 0x18000 DynamicSection2
0x40000 0x40000 Reserved
*/

#pragma pack(push, 1)
typedef struct _joystick_calibration_data
{
    uint16_t AnalogStickCalXPositive : 12;
    uint16_t AnalogStickCalYPositive : 12;
    uint16_t AnalogStickCalX0 : 12;
    uint16_t AnalogStickCalY0 : 12;
    uint16_t AnalogStickCalXNegative : 12;
    uint16_t AnalogStickCalYNegative : 12;
} joystick_calibration_data;
// why uint8 for x & uint16 for y???
// ans:they r uint12
typedef struct _user_joystick_calibration_data
{
    uint16_t AnalogStickLeftUserMagicNumber;
    joystick_calibration_data AnalogStickLeftUserCalibrationValue;
    uint16_t AnalogStickRightUserMagicNumber;
    joystick_calibration_data AnalogStickRightUserCalibrationValue;
} user_joystick_calibration_data;
typedef struct _factory_joystick_calibration_data
{
    joystick_calibration_data AnalogStickLeftFactoryCalibrationValue;
    joystick_calibration_data AnalogStickRightFactoryCalibrationValue;
} factory_joystick_calibration_data;
typedef struct _imu_calibration_data
{
    int16_t Accelerometer0OffsetX;
    int16_t Accelerometer0OffsetY;
    int16_t Accelerometer0OffsetZ;
    uint16_t Accelerometer1gScaleX;
    uint16_t Accelerometer1gScaleY;
    uint16_t Accelerometer1gScaleZ;
    int16_t Gyroscope0OffsetX;
    int16_t Gyroscope0OffsetY;
    int16_t Gyroscope0OffsetZ;
    uint16_t Gyroscope78rpmScaleX;
    uint16_t Gyroscope78rpmScaleY;
    uint16_t Gyroscope78rpmScaleZ;
} imu_calibration_data;
typedef struct _user_imu_calibration_data
{
    uint16_t SixAxisUserCalibrationMagicNumber;
    imu_calibration_data SixAxisSensorCalibrationValue;
} user_imu_calibration_data;
typedef struct _user_calibration_data
{
    union
    {
        uint8_t Reserved1[16];
        struct
        {
            uint8_t nonexist;
            uint16_t internal_center[4];
            uint8_t tag;
            uint8_t tag2;
            uint8_t reserved[5];
        };
    };
    user_joystick_calibration_data UserJoystickCalibrationValue;
    user_imu_calibration_data UserSixAxisSensorCalibrationValue;
    union
    {
        uint8_t Reserved2[8];
        struct
        {
            uint32_t mcause;
            uint32_t mepc;
        };
    };
} user_calibration_data;
typedef struct _rgb_data_complete
{
    union
    {
        struct
        {
            uint8_t b;
            uint8_t g;
            uint8_t r;
        };
        uint32_t load : 24;
    };
} rgb_data_complete;
typedef struct _controller_color_data
{
    rgb_data_complete MainColor;
    rgb_data_complete SubColor;
    rgb_data_complete ExtraColor1;
    rgb_data_complete ExtraColor2;
} controller_color_data;
typedef struct _device_design_data
{
    controller_color_data ControllerColor;
    uint8_t DesignVariation; // 0,1,2
} device_design_data;
typedef struct _imu_model_data
{
    uint16_t SixAxisHorizontalOffsetX;
    uint16_t SixAxisHorizontalOffsetY;
    uint16_t SixAxisHorizontalOffsetZ;
} imu_model_data;
typedef struct _joystick_model_value
{
    /*uint8_t AnalogStickModelNoise;
    uint16_t AnalogStickModelTypicalStroke;
    uint8_t AnalogStickModelCenterDeadZoneSize;
    uint16_t AnalogStickModelCircuitDeadZoneScale;
    uint8_t AnalogStickModelMinimumStrokeXPositive;
    uint16_t AnalogStickModelMinimumStrokeYPositive;
    uint8_t AnalogStickModelMinimumStrokeXNegative;
    uint16_t AnalogStickModelMinimumStrokeYNegative;
    uint8_t AnalogStickModelCenterRangeXPositive;
    uint16_t AnalogStickModelCenterRangeYPositive;
    uint8_t AnalogStickModelCenterRangeXNegative;
    uint16_t AnalogStickModelCenterRangeYNegative;*/
    uint16_t AnalogStickModelNoise : 12;
    uint16_t AnalogStickModelTypicalStroke : 12;
    uint16_t AnalogStickModelCenterDeadZoneSize : 12;
    uint16_t AnalogStickModelCircuitDeadZoneScale : 12;
    uint16_t AnalogStickModelMinimumStrokeXPositive : 12;
    uint16_t AnalogStickModelMinimumStrokeYPositive : 12;
    uint16_t AnalogStickModelMinimumStrokeXNegative : 12;
    uint16_t AnalogStickModelMinimumStrokeYNegative : 12;
    uint16_t AnalogStickModelCenterRangeXPositive : 12;
    uint16_t AnalogStickModelCenterRangeYPositive : 12;
    uint16_t AnalogStickModelCenterRangeXNegative : 12;
    uint16_t AnalogStickModelCenterRangeYNegative : 12;
} joystick_model_value;
typedef struct _model_data
{
    imu_model_data SixAxisSensorModelValue;
    joystick_model_value AnalogStickMainModelValue;
} model_data;
typedef struct _factory_configuration_data
{
    uint8_t IdentificationCode[16]; // sn code
    uint8_t Reserved1[2];
    uint8_t DeviceType; // 0x03 to be a pro controller
    uint8_t BoardRevision;
    uint8_t Reserved2[7];
    uint8_t FormatVersion;
    uint8_t Reserved3[4];
    imu_calibration_data SixAxisSensorCalibrationValue;
    uint8_t Reserved4[5];
    factory_joystick_calibration_data JoystickCalibrationValue;
    uint8_t Reserved5;
    device_design_data Design;
    uint8_t Reserved6[35];
    model_data Model;
    joystick_model_value AnalogStickSubModelValue;
    uint8_t Reserved7;
    uint8_t AccelerometerAxisAssignment;
    uint8_t GyroscopeAxisAssignment;
    uint8_t AnalogStickMainAxisAssignment;
    uint8_t AnalogStickSubAxisAssignment;
    // uint8_t Reserved8[337];
    uint16_t BatteryVoltage;
    /*
    0x202   0xB7E   Reserved
    0xD80   0x2 TarragonVid
    0xD82   0x2 TarragonPid
    0xD84   0x7C    Reserved
    0xE00   0x100   InspectionLog
    0xF00   0x100   Reserved
    */
} factory_configuration_data;
typedef struct _factory_configuration_flash_pack
{
    uint8_t nonexist;   // available when 0
    uint8_t DeviceType; // 0x03 to be a pro controller
    uint8_t BoardRevision;
    uint8_t FormatVersion;
    imu_calibration_data SixAxisSensorCalibrationValue;
    factory_joystick_calibration_data JoystickCalibrationValue;
    device_design_data Design;
    model_data Model;
    joystick_model_value AnalogStickSubModelValue;
    /*uint8_t AccelerometerAxisAssignment;
    uint8_t GyroscopeAxisAssignment;
    uint8_t AnalogStickMainAxisAssignment;
    uint8_t AnalogStickSubAxisAssignment;
    uint16_t BatteryVoltage;*/
} factory_configuration_flash_pack;
typedef struct _rgb_data_simple
{
    uint8_t b : 4;
    uint8_t g : 4;
    uint8_t r : 4;
} rgb_data_simple;
// used for wired model to save space
typedef struct _user_config_data
{
    // uint8_t usb_report_interval;
    // uint8_t joystick_correction[16];
    union
    {
        uint8_t config_bitmap1;
        struct
        {
            uint8_t nonexist : 1;
            uint8_t led_disabled : 1;
            uint8_t home_led_typ : 1;
            uint8_t x_y_swap : 1;
            uint8_t a_b_swap : 1;
            uint8_t rumble_disabled : 1;
            uint8_t rumble_pattern : 1;
            uint8_t imu_disabled : 1;
        };
    };
    uint8_t in_interval;
    uint8_t out_interval;
    uint8_t hd_rumble_amp_ratio[4];
    int8_t joystick_ratio[4];
    // uint8_t reserved[2];
    uint16_t imu_sample_gap;
    uint16_t joystick_snapback_deadzone[2];

    // raw center+offset=2048
    // offset=2048-raw_center
    uint16_t joystick_snapback_filter_max_delay;
    // 0:no filter 1:liner filter 2:power filter 3:force center when across
    uint8_t bd_addr[ESP_BD_ADDR_LEN];
    uint8_t imu_ratio_x; // div 127
    uint8_t imu_ratio_y; // div 127
    uint8_t imu_ratio_z; // div 127
    uint8_t pro_fw_version;
    uint8_t ns_pkt_timer_mode; // 0:stock(timestamp_div_5) 1:timestamp 2:pkt cnt
    uint8_t dead_zone[4];
    uint8_t dead_zone_mode;
    uint8_t rgb_cnt;
    rgb_data_complete rgb_data[RGB_MAX_CNT];
} user_config_data;
// 117 byte now
#pragma pack(pop)

#define JOYSITCK_FITTING_PARAM_RATIO (255)
#define JOYSITCK_FITTING_PARAM_RATIO_SHIFT (8)

extern factory_configuration_data factory_configuration;
extern user_calibration_data user_calibration;
extern user_config_data user_config;
extern uint32_t joystick_snapback_deadzone_sq[2];

void conf_init();
void conf_read(uint32_t addr, uint8_t *buf, uint8_t size);
uint8_t conf_write(uint32_t addr, uint8_t *buf, uint8_t size);
void custom_conf_read();
uint8_t custom_conf_write();
void conf_flush();
void fac_conf_read();
uint8_t fac_conf_write();
void uart_config_cb(uart_packet *p);

#endif /* USER_CONF_H_ */
