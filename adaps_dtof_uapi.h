/*
 * Copyright (C) 2023-2024 Adaps Photonics Inc.
 */

#ifndef _ADAPS_DTOF_UAPI_H
#define _ADAPS_DTOF_UAPI_H

#include <linux/types.h>

#define ADAPS_DTOF_PRIVATE              0x1000

#define ADAPS_ROI_SRAM_DATA_CNT         2

#define RKMODULE_MAX_SENSOR_MODE        10
#define RKMODULE_MODE_NAME_MAX          8

#define SOC_PLATFORM_ROCKCHIP           0x0000
#define SOC_PLATFORM_RK3568             (SOC_PLATFORM_ROCKCHIP + 1)

#define SOC_PLATFORM_TI                 0x1000
#define SOC_PLATFORM_TI_AM62A           (SOC_PLATFORM_TI + 1)

#if !defined(BIT)
#define BIT(n)                  (1 << n)
#endif

typedef enum {
    MIPI_TWO_DT_MODE = 0,
    MIPI_ONE_DT_MODE,

    MIPI_DT_MODE_COUNT
} mipi_datatype_mode_t;

typedef enum {
    SYS_CLOCK_330M_HZ = 330,
    SYS_CLOCK_250M_HZ = 250,
    SYS_CLOCK_200M_HZ = 200,
}sys_clock_freq_t;

typedef enum {
    MIPI_CLOCK_CONTINUOUS = 0,
    MIPI_CLOCK_NON_CONTINUOUS,
}mipi_clock_t;

typedef enum {
    MIPI_DATA_LANES_4 = 4,             // data lane 0 -- 3
    MIPI_DATA_LANES_2 = 2,             // data lane 0 -- 1
}mipi_data_lane_t;

typedef enum {
    MIPI_SPEED_1G_BPS       = 1000, // default value, available for ads6311 and ads6401
    MIPI_SPEED_1G5_BPS      = 1500, // available for ads6311 and ads6401
    MIPI_SPEED_1G2_BPS      = 1200, // available for ads6311 and ads6401
    MIPI_SPEED_800M_BPS     = 800,  // available for ads6311 only now
    MIPI_SPEED_720M_BPS     = 720,  // available for ads6401 only now
    MIPI_SPEED_600M_BPS     = 600,  // NOT availabe now
    MIPI_SPEED_500M_BPS     = 500,  // available for ads6401 only now
    MIPI_SPEED_200M_BPS     = 200,  // available for ads6401 only now
    MIPI_SPEED_166M64_BPS   = 166,  // NOT availabe now
}mipi_speed_per_lane_t;

struct sys_perf_param {
    sys_clock_freq_t           sys_clk_freq;
    mipi_clock_t               mipi_clk_type;
    mipi_speed_per_lane_t      mipi_speed_per_lane;
    mipi_data_lane_t           mipi_data_lane_cnt;
}__attribute__ ((packed));


#if 1 //def __KERNEL__
#if defined(CONFIG_VIDEO_ADS6311)  // FOR ADAPS_HAWK

#define SENSOR_OTP_DATA_SIZE    0x20 // unit is word (16-bits)

struct rkmodule_sensor_mode {
    char modename[RKMODULE_MODE_NAME_MAX];
    __u32 image_width;
    __u32 image_height;
    __u16 userdata_width;
    __u16 userdata_height;
    __u32 image_datatype;
    __u32 userdata_datatype;
} __attribute__ ((packed));

struct rkmodule_sensor_module_inf {
    struct rkmodule_sensor_mode  sensor_mode[RKMODULE_MAX_SENSOR_MODE];
    __u8 mode_size;
} __attribute__ ((packed));

typedef struct {
    __u16 addr;
    __u16 val;
} reg16_data16_t;

typedef struct {
    __u16 addr;
    __u8 reserved;
    __u8 val;
} reg16_data8_t;

#if defined(NEWER_ADS6311_DRV_COWORK_WITH_OLDER_APP) // if use the new ads6311.c driver co-work with old user space App whose release version <= v3.2.0
typedef struct {
    __u32 workMode;
    __u16 subFrameNum;
    __u16 regSet1;
    __u16 regSet2;
    __u16 roiSram;
} enable_script_t;

#else

typedef struct {
    __u32 workMode;
    __u16 subFrameNum;
    __u16 regSet1;
    __u16 regSet2;
    __u16 roiSram[ADAPS_ROI_SRAM_DATA_CNT];
} enable_script_t;
#endif

typedef enum hawk_soc_board_version {
    HAWK_SOC_BOARD_VER_UNKNOWN = 0,
    HAWK_SOC_BOARD_VER_DEMO3   = SOC_PLATFORM_RK3568,       // 0x0001
    HAWK_SOC_BOARD_VER_MINI_DEMO4,

    HAWK_SOC_BOARD_VER_TI_AM62A   = SOC_PLATFORM_TI_AM62A,  // 0x1001
}hawk_soc_board_version_t;

typedef enum hawk_chip_eco_version {
    HAWK_CHIP_ECO_VER_UNKNOWN = 0,
    HAWK_CHIP_ECO_VER_NTO,  // NewTapeOut
    HAWK_CHIP_ECO_VER_ECO1,
    HAWK_CHIP_ECO_VER_ECO2,
    HAWK_CHIP_ECO_VER_ECO3,
    HAWK_CHIP_ECO_VER_ECO4,
    HAWK_CHIP_ECO_VER_ECO5,
    HAWK_CHIP_ECO_VER_ECO6,
    HAWK_CHIP_ECO_VER_ECO7,
    HAWK_CHIP_ECO_VER_ECO8,
}hawk_chip_eco_version_t;

typedef enum hawk_lens_type {
    HAWK_LENS_WIDEANGLE = 0,
    HAWK_LENS_TELEPHOTO,

    HAWK_LENS_COUNT
}hawk_lens_t;

typedef enum hawk_vcsel_op_code {
    HAWK_VCSEL_OP_UNININT = 0,
    HAWK_VCSEL_OP_START = 1,
    HAWK_VCSEL_OP_STOP = 2,
}hawk_vcsel_op_code_t;

enum hawk_rx_error_type_e {
    HAWK_RX_FSM_ALARM               = BIT(0),  // internal state-machine mismatch alarm
    HAWK_RX_WDT_ALARM               = BIT(1),  // watch-dog timer overflow alarm
    HAWK_RX_DV11_ALARM              = BIT(2),  // DV11 voltage abnormal alarm
    HAWK_RX_AV11_ALARM              = BIT(3),  // AV11 voltage abnormal alarm
    HAWK_RX_SV11_ALARM              = BIT(4),  // SV11 voltage abnormal alarm
    HAWK_RX_AV33_ALARM              = BIT(5),  // DV33 voltage abnormal alarm
    HAWK_RX_NOT_APPLIED6            = BIT(6),  // not applied
    HAWK_RX_NOT_APPLIED7            = BIT(7),  // not applied

    HAWK_RX_NOT_APPLIED8            = BIT(8),  // not applied
    HAWK_RX_NOT_APPLIED9            = BIT(9),  // not applied
    HAWK_RX_TRIGO0_ALARM            = BIT(10),  // triger o0 alarm
    HAWK_RX_TRIGO1_ALARM            = BIT(11),  // triger o1 alarm
    HAWK_RX_TRIGO2_ALARM            = BIT(12),  // triger o2 alarm
    HAWK_RX_TRIGO3_ALARM            = BIT(13),  // triger o3 alarm
    HAWK_RX_DRV_ERR_ALARM           = BIT(14),  // tx driver error alarm
    HAWK_RX_RESERVED                = BIT(15),  // reserved

    HAWK_RX_LOW_TEMP_OTP_DATA_ERR   = BIT(16),  // Low temperature otp parameter has some error
    HAWK_RX_NORMAL_OTP_DATA_ERR     = BIT(17),  // Normal otp parameter has some error
    HAWK_RX_HIGH_OTP_DATA_ERR       = BIT(18),  // High temperature otp parameter has some error
};

typedef enum hawk_pvdd_switch_strategy_e {
    HAWK_PVDD_SWITCH_AUTO               = 0,    // strong <-> weak PVDD switch cycle if two ROI sram data are available, otherwize use strong PVDD if only the first ROI sram is available.
    HAWK_PVDD_SWITCH_FORCE_TO_STRONG    = 1,    // use strong PVDD although the second ROI sram is available.
    HAWK_PVDD_SWITCH_FORCE_TO_WEAK      = 2,    // use weak PVDD if the second ROI sram is available.
    HAWK_PVDD_SWITCH_STRATEGY_CNT
} pvdd_swt_strategy_t;

enum {
    HAWK_OTP_TEMPERATURE_PARAM_LOW,
    HAWK_OTP_TEMPERATURE_PARAM_ROOM,
    HAWK_OTP_TEMPERATURE_PARAM_HIGH,
    HAWK_OTP_TEMPERATURE_PARAM_COUNT,
};

enum hawk_otp_checksum_state{
    HAWK_OTP_CHECKSUM_UNSAVED,     // only for these BLANK chip without CP test
    HAWK_OTP_CHECKSUM_MATCHED,
    HAWK_OTP_CHECKSUM_MISMATCHED,
};

struct hawk_otp_temperature_param
{
    __u8 chksum_state;              // refer to the enum hawk_otp_checksum_state
    __u16 vbe_code;
    __u16 vbr_voltage;              // value x10 mv
    __u8 temperature;               // low temperature use signed 8-bit integer, while room and high temperature use unsigned temperature
}__attribute__ ((packed));

struct hawk_otp_key_data
{
    u16 chip_eco_version;
    struct hawk_otp_temperature_param temperature_param[HAWK_OTP_TEMPERATURE_PARAM_COUNT];
}__attribute__ ((packed));

struct hawk_otp_ref_voltage
{
    bool otp_blank;
    bool otp_read;   // whether read the otp data or not?
    u16 adc_reference_voltage;      // unit is mV
}__attribute__ ((packed));

struct hawk_otp_raw_data
{
    bool otp_blank;
    bool otp_read;   // whether read the otp data or not?
    u16 raw_data[SENSOR_OTP_DATA_SIZE];
}__attribute__ ((packed));

struct hawk_norflash_op_param
{
    __u32 op_code; //0:read,1:write
    __u32 offset;
    __u32 len;
}__attribute__ ((packed));

struct hawk_sensor_cfg_data
{
    __u8 hVldSeg;
    __u8 vRollNum;
    __u8 hRollNum;
    __u8 scanMode;
}__attribute__ ((packed));

#define ADTOF_ENABLE_STREAM_NUM      \
    _IOW('T', ADAPS_DTOF_PRIVATE + 2, __u32 *)

#define ADTOF_ENABLE_SCRIPT_START      \
    _IOW('T', ADAPS_DTOF_PRIVATE + 3, enable_script_t *)

#define ADTOF_SET_VCSEL_CHANNEL      \
    _IOW('T', ADAPS_DTOF_PRIVATE + 4, __u16 *)

#define ADTOF_SET_SENSOR_REGISTER       \
    _IOW('T', ADAPS_DTOF_PRIVATE + 5, reg16_data8_t *)

#define ADTOF_GET_SENSOR_REGISTER       \
    _IOR('T', ADAPS_DTOF_PRIVATE + 6, reg16_data8_t *)

#define ADTOF_SET_MCU_REGISTER       \
    _IOW('T', ADAPS_DTOF_PRIVATE + 7, reg16_data16_t *)

#define ADTOF_GET_MCU_REGISTER       \
    _IOR('T', ADAPS_DTOF_PRIVATE + 8, reg16_data16_t *)

#define ADTOF_GET_DRIVER_VERSION      \
    _IOR('T', ADAPS_DTOF_PRIVATE + 9, __u32 *)

#define ADTOF_GET_MCU_RUNNING_FW_TYPE \
    _IOW('T', ADAPS_DTOF_PRIVATE + 10, __u16 *)             // 0x0A

#define ADTOF_GET_MCUFIRMWARE_VERSION \
    _IOR('T', ADAPS_DTOF_PRIVATE + 11, __u16 *)             // 0x0B

#define ADTOF_LET_MCU_ENTER_BOOTLOADER      \
    _IO('T', ADAPS_DTOF_PRIVATE + 12)                       // 0x0C

#define ADTOF_SET_MCU_FW_CRC32      \
    _IOW('T', ADAPS_DTOF_PRIVATE + 13, __u32 *)             // 0x0D

#define ADTOF_SET_MCU_FW_LENGTH      \
    _IOW('T', ADAPS_DTOF_PRIVATE + 14, __u32 *)             // 0x0E

#define ADTOF_GET_BLOCK_SIZE_4_FLASH_WRITE      \
    _IOR('T', ADAPS_DTOF_PRIVATE + 15, __u32 *)             // 0x0F

#define ADTOF_TRANSFER_DATA_WITH_OFFSET      \
    _IOW('T', ADAPS_DTOF_PRIVATE + 16, __u32 *)             // 0x10

#define ADTOF_GET_FLASH_WRITE_STATUS \
    _IOW('T', ADAPS_DTOF_PRIVATE + 17, __u16 *)             // 0x11

#define ADTOF_SET_FLASH_WRITE_STATUS \
    _IOW('T', ADAPS_DTOF_PRIVATE + 18, __u16 *)             // 0x12

#define ADTOF_GET_MCU_FW_UPGRADE_STATUS \
    _IOW('T', ADAPS_DTOF_PRIVATE + 19, __u16 *)             // 0x13

#define ADTOF_LET_MCU_REBOOT      \
    _IO('T', ADAPS_DTOF_PRIVATE + 20)                       // 0x14

#define ADTOF_GET_MCUHARDWARE_VERSION \
    _IOR('T', ADAPS_DTOF_PRIVATE + 21, __u16 *)             // 0x15

//for hawk sensor norflash data
#define ADTOF_NORFLASH_OPERATION       \
    _IOW('T', ADAPS_DTOF_PRIVATE + 22, struct hawk_norflash_op_param *)     // 0x16

#define ADTOF_VCSEL_OPERATION       \
    _IOW('T', ADAPS_DTOF_PRIVATE + 23, __u32 *)             // 0x17

#define ADTOF_SET_VOP_VOLTAGE      \
    _IOW('T', ADAPS_DTOF_PRIVATE + 24, __s16 *)             // 0x18

#define ADTOF_SET_PVDD_VOLTAGE      \
    _IOW('T', ADAPS_DTOF_PRIVATE + 25, __u16 *)             // 0x19

#define ADTOF_GET_TX_TEMPERATURE      \
    _IOR('T', ADAPS_DTOF_PRIVATE + 26, __u16 *)             // 0x1A

#define ADTOF_GET_RX_TEMPERATURE      \
    _IOR('T', ADAPS_DTOF_PRIVATE + 27, __s16 *)             // 0x1B

#define ADTOF_GET_HAWK_CHIP_ECO_VERSION      \
    _IOR('T', ADAPS_DTOF_PRIVATE + 28, __u16 *)             // 0x1C

#define ADTOF_GET_HAWK_CHIP_ASC_ALARM      \
    _IOR('T', ADAPS_DTOF_PRIVATE + 29, __u32 *)             // 0x1D

#define ADTOF_GET_HAWK_OTP_KEY_DATA       \
    _IOR('T', ADAPS_DTOF_PRIVATE + 30, struct hawk_otp_key_data *)          // 0x1E

#define ADTOF_SET_LENS_TYPE      \
    _IOW('T', ADAPS_DTOF_PRIVATE + 31, __u8 *)              // 0x1F

#define ADTOF_SWITCH_ROI_SRAM      \
    _IO('T', ADAPS_DTOF_PRIVATE + 32)                       // 0x20

#define ADTOF_GET_SOC_BOARD_VERSION      \
    _IOR('T', ADAPS_DTOF_PRIVATE + 33, __u16 *)             // 0x21

#define ADTOF_GET_HAWK_OTP_REF_VOLTAGE       \
    _IOR('T', ADAPS_DTOF_PRIVATE + 34, struct hawk_otp_ref_voltage *)          // 0x22

#define ADTOF_SET_MCU_ERR_REPORT_MASK      \
    _IOW('T', ADAPS_DTOF_PRIVATE + 35, __u16 *)             // 0x23

#define ADTOF_SET_PVDD_VOLTAGE_4_WEAK_LASER      \
    _IOW('T', ADAPS_DTOF_PRIVATE + 36, __u16 *)             // 0x24

#define ADTOF_SET_MIPI_DATATYPE_MODE      \
    _IOW('T', ADAPS_DTOF_PRIVATE + 37, __u8 *)              // 0x25

#define ADTOF_GET_MIPI_DATATYPE_MODE       \
    _IOR('T', ADAPS_DTOF_PRIVATE + 38, __u8 *)          // 0x26

#define ADTOF_SET_SYS_PERF_PARAMETERS      \
    _IOW('T', ADAPS_DTOF_PRIVATE + 39, struct sys_perf_param *)              // 0x27

#define ADTOF_GET_SYS_PERF_PARAMETERS       \
    _IOR('T', ADAPS_DTOF_PRIVATE + 40, struct sys_perf_param *)          // 0x28

#define ADTOF_SET_EXPO_PBRS_ENABLE      \
    _IOW('T', ADAPS_DTOF_PRIVATE + 41, bool *)              // 0x29

#define ADTOF_SET_ASC_ENABLE_CFG      \
    _IOW('T', ADAPS_DTOF_PRIVATE + 42, __u8 *)              // 0x2A

#define ADTOF_SET_PVDD_SWITCH_STRATEGY      \
    _IOW('T', ADAPS_DTOF_PRIVATE + 43, __u8 *)              // 0x2B   see also pvdd_swt_strategy_t

#define ADTOF_GET_HAWK_OTP_RAW_DATA       \
    _IOR('T', ADAPS_DTOF_PRIVATE + 44, struct hawk_otp_raw_data *)          // 0x2C

#define ADTOF_SET_SENSOR_CFG_DATA       \
    _IOW('T', ADAPS_DTOF_PRIVATE + 45, struct hawk_sensor_cfg_data *)          // 0x2D
#endif // FOR ADAPS_HAWK



//==========================================================================
#if defined(CONFIG_VIDEO_ADS6401) || defined(CONFIG_VIDEO_PACIFIC) // FOR ADAPS_SWIFT

#include "adaps_types.h"

#define MAX_CALIB_SRAM_ROTATION_GROUP_CNT   9

#define FW_VERSION_LENGTH                   12

// There are two group Calibration SRAM for spod address, every group has 4 calibration registers.
#define CALIB_SRAM_REG_BASE0                0xFB
#define CALIB_SRAM_REG_BASE1                0xF7
#define CSRU_SRAM_OFFSET_REG                0xC7

#define PER_ZONE_MAX_SPOT_COUNT             240

#define PER_CALIB_SRAM_ZONE_SIZE            512     //unit is bytes, actual use size is 480 bytes, the remaining 32 bytes is 0
#define ZONE_COUNT_PER_SRAM_GROUP           4
#define CALIB_SRAM_GROUP_COUNT              2

#define ROI_SRAM_BUF_MAX_SIZE               (2*1024)   // 2x1024, unit is bytes
#define REG_SETTING_BUF_MAX_SIZE_PER_SEG    2048       // unit is bytes, there may be 2 segments

#define PER_ROISRAM_GROUP_SIZE              (PER_CALIB_SRAM_ZONE_SIZE * ZONE_COUNT_PER_SRAM_GROUP) //unit is bytes

#define ALL_ROISRAM_GROUP_SIZE              (PER_CALIB_SRAM_ZONE_SIZE * ZONE_COUNT_PER_SRAM_GROUP * CALIB_SRAM_GROUP_COUNT)

#define SWIFT_MAX_SPOT_COUNT                (PER_ZONE_MAX_SPOT_COUNT * ZONE_COUNT_PER_SRAM_GROUP * CALIB_SRAM_GROUP_COUNT)

#define SWIFT_PRODUCT_ID_SIZE               12

#define OFFSET(structure, member)           ((uintptr_t)&((structure*)0)->member)
#define MEMBER_SIZE(structure, member)      sizeof(((structure*)0)->member)

#pragma pack(1)
typedef struct WalkErrorParameters {
    uint8_t zoneId;
    uint8_t spotId;
    uint8_t x;
    uint8_t y;
    float paramD;
    float paramX;
    float paramY;
    float paramZ;
    float param0;
    uint8_t dummy1;//reserved
    uint8_t dummy2;//reserved
}WalkErrorParam_t;
#pragma pack()

// -------------------- swift SPOT module definition start -------------
enum {
    CALIBRATION_INFO,
    SRAM_DATA,
    INTRINSIC,
    OUTDOOR_OFFSET,
    SPOT_OFFSET_B,
    SPOT_OFFSET_A,
    TDC_DELAY,
    REF_DISTANCE,
    PROXIMITY,
    HOTPIXEL_DEADPIXEL,
    WALKERROR,
    SPOT_ENERGY,
    NOISE,
    RESERVED,
    RESERVED2,
    CHECKSUM_ALL = 19
};

#define SPOT_MODULE_DEVICE_NAME_LENGTH                  64
#define SPOT_MODULE_INFO_LENGTH                         16
#define SPOT_MODULE_OFFSET_SIZE                         960

#define SPOT_MODULE_WALKERROR_SIZE                      (sizeof(WalkErrorParam_t) * PER_ZONE_MAX_SPOT_COUNT *ZONE_COUNT_PER_SRAM_GROUP) //26 * 960
#define SPOT_MODULE_SPOTENERGY_SIZE                     960
#define SPOT_MODULE_NOISE_SIZE                          10732
#define SPOT_MODULE_CHECKSUM_SIZE                       20
#define SPOT_MODULE_MODULE_INFO_RESERVED_SIZE           176
#define SPOT_MODULE_WALKERROR_RESERVED_SIZE             5760

#define SPOT_MODULE_EEPROM_CAPACITY_SIZE                (64*1024)  // 64*1024, unit is bytes
#define SPOT_MODULE_EEPROM_PAGE_SIZE                    64

#pragma pack(4)
typedef struct SwiftSpotModuleEepromData
{
    // In Swift EEPROM, one page has 64 bytes.
    // Page 1
    char            deviceName[SPOT_MODULE_DEVICE_NAME_LENGTH]; // Calibration version infomation
    // Page 2 - 65
    unsigned char   sramData[ALL_ROISRAM_GROUP_SIZE];
    // Page 66
    float           intrinsic[9];     // 36 bytes
    float           offset;           // 4 bytes
    __u32           refSpadSelection; // 4 bytes
    __u32           driverChannelOffset[4];  // 16 bytes
    __u32           distanceTemperatureCoefficient;
    // Page 67 - 186
    float           spotPos[SWIFT_MAX_SPOT_COUNT]; // 7680 bytes, 120 pages, reused for offset B and outdoor
    // Page 187 - 246
    float           spotOffset[SPOT_MODULE_OFFSET_SIZE]; // 3840 bytes, 60 pages, offset A
    // Page 247
    __u32           tdcDelay[16]; // 64 bytes
    // Page 248
    float           indoorCalibTemperature; // Calibration temperature.
    float           indoorCalibRefDistance; // Calibration reference distance.
    float           outdoorCalibTemperature; // Calibration temperature.
    float           outdoorCalibRefDistance; // Calibration reference distance.
    float           calibrationInfo[12]; // reserved
    // Page 249 - 252 
    uint8_t         pxyHistogram[248]; // pxy histogram
    float           pxyDepth;          // pxy depth
    float           pxyNumberOfPulse;  // pxy number of pulse
    // Page 253
    uint16_t        markedPixels[32];  // 16 hot pixels, 16 dead pixels.
    // Page 254
    char            moduleInfo[SPOT_MODULE_INFO_LENGTH]; // adaps calibration info.
    //Page  255-256
    char            reserved1[SPOT_MODULE_MODULE_INFO_RESERVED_SIZE];
    //Page  257-622
    char            WalkError[SPOT_MODULE_WALKERROR_SIZE];
    //Page  623-736
    char            reserved2[SPOT_MODULE_WALKERROR_RESERVED_SIZE];
    //Page  737-796           
    float           SpotEnergy[SPOT_MODULE_SPOTENERGY_SIZE];

    float           RawDepthMean[SPOT_MODULE_SPOTENERGY_SIZE];
    //Page  796-1023
    char            noise[SPOT_MODULE_NOISE_SIZE];
    //Page  1024
    uint8_t            checksum[SPOT_MODULE_CHECKSUM_SIZE];
}swift_spot_module_eeprom_data_t;
#pragma pack()

#define  ADS6401_EEPROM_VERSION_INFO_OFFSET                  OFFSET(swift_spot_module_eeprom_data_t, deviceName)               /// 0, 0x00
#define  ADS6401_EEPROM_VERSION_INFO_SIZE                    MEMBER_SIZE(swift_spot_module_eeprom_data_t, deviceName)

// please refer to data struct 'swift_eeprom_data_t' in SpadisPC
#define  ADS6401_EEPROM_ROISRAM_DATA_OFFSET                  OFFSET(swift_spot_module_eeprom_data_t, sramData)               /// 64, 0x40
#define  ADS6401_EEPROM_ROISRAM_DATA_SIZE                    MEMBER_SIZE(swift_spot_module_eeprom_data_t, sramData)

#define  ADS6401_EEPROM_INTRINSIC_OFFSET                     OFFSET(swift_spot_module_eeprom_data_t, intrinsic)              /// 4160 0x1040
#define  ADS6401_EEPROM_INTRINSIC_SIZE                       MEMBER_SIZE(swift_spot_module_eeprom_data_t, intrinsic)         /// 9xsizeof(float)

#define  ADS6401_EEPROM_ACCURATESPODPOS_OFFSET               OFFSET(swift_spot_module_eeprom_data_t, spotPos)        /// 4224 0x1080
#define  ADS6401_EEPROM_ACCURATESPODPOS_SIZE                 MEMBER_SIZE(swift_spot_module_eeprom_data_t, spotPos)   /// 4x240x2xsizeof(float)=1920x4=7680

#define  ADS6401_EEPROM_SPOTOFFSET_OFFSET                    OFFSET(swift_spot_module_eeprom_data_t, spotOffset)             /// 11904 0x2e80
#define  ADS6401_EEPROM_SPOTOFFSET_SIZE                      MEMBER_SIZE(swift_spot_module_eeprom_data_t, spotOffset)        /// 4x240xsizeof(float)=960x4=3840

#define  ADS6401_EEPROM_TDCDELAY_OFFSET                      OFFSET(swift_spot_module_eeprom_data_t, tdcDelay)               /// 15744 0x3d80 
#define  ADS6401_EEPROM_TDCDELAY_SIZE                        MEMBER_SIZE(swift_spot_module_eeprom_data_t, tdcDelay)          /// 2xsizeof(uint32_t)=16x4=64

#define  ADS6401_EEPROM_INDOOR_CALIBTEMPERATURE_OFFSET       OFFSET(swift_spot_module_eeprom_data_t, indoorCalibTemperature)               /// 15808 0x3dc0
#define  ADS6401_EEPROM_INDOOR_CALIBTEMPERATURE_SIZE         MEMBER_SIZE(swift_spot_module_eeprom_data_t, indoorCalibTemperature)          /// 1xsizeof(float)=1x4=4

#define  ADS6401_EEPROM_INDOOR_CALIBREFDISTANCE_OFFSET       OFFSET(swift_spot_module_eeprom_data_t, indoorCalibRefDistance)               /// 15812 0x3dc4
#define  ADS6401_EEPROM_INDOOR_CALIBREFDISTANCE_SIZE         MEMBER_SIZE(swift_spot_module_eeprom_data_t, indoorCalibRefDistance)          /// 1xsizeof(float)=1x4=4

#define  ADS6401_EEPROM_OUTDOOR_CALIBTEMPERATURE_OFFSET      OFFSET(swift_spot_module_eeprom_data_t, outdoorCalibTemperature)               /// 15816
#define  ADS6401_EEPROM_OUTDOOR_CALIBTEMPERATURE_SIZE        MEMBER_SIZE(swift_spot_module_eeprom_data_t, outdoorCalibTemperature)          /// 1xsizeof(float)=1x4=4

#define  ADS6401_EEPROM_OUTDOOR_CALIBREFDISTANCE_OFFSET      OFFSET(swift_spot_module_eeprom_data_t, outdoorCalibRefDistance)               /// 15820
#define  ADS6401_EEPROM_OUTDOOR_CALIBREFDISTANCE_SIZE        MEMBER_SIZE(swift_spot_module_eeprom_data_t, outdoorCalibRefDistance)          /// 1xsizeof(float)=1x4=4

#define  ADS6401_EEPROM_CALIBRATIONINFO_OFFSET              OFFSET(swift_spot_module_eeprom_data_t, calibrationInfo)
#define  ADS6401_EEPROM_CALIBRATIONINFO_SIZE                MEMBER_SIZE(swift_spot_module_eeprom_data_t, calibrationInfo)

#define  ADS6401_EEPROM_PROX_HISTOGRAM_OFFSET                OFFSET(swift_spot_module_eeprom_data_t, pxyHistogram)               /// 15824+12x(sizeof(float))=15872
#define  ADS6401_EEPROM_PROX_HISTOGRAM_SIZE                  MEMBER_SIZE(swift_spot_module_eeprom_data_t, pxyHistogram)          /// 256

#define  ADS6401_EEPROM_PROX_DEPTH_OFFSET                   OFFSET(swift_spot_module_eeprom_data_t, pxyDepth)
#define  ADS6401_EEPROM_PROX_DEPTH_SIZE                     MEMBER_SIZE(swift_spot_module_eeprom_data_t, pxyDepth)

#define  ADS6401_EEPROM_PROX_NO_OF_PULSE_OFFSET              OFFSET(swift_spot_module_eeprom_data_t, pxyNumberOfPulse)
#define  ADS6401_EEPROM_PROX_NO_OF_PULSE_SIZE                MEMBER_SIZE(swift_spot_module_eeprom_data_t, pxyNumberOfPulse)

#define  ADS6401_EEPROM_MARKED_PIXELS_OFFSET                   OFFSET(swift_spot_module_eeprom_data_t, markedPixels)
#define  ADS6401_EEPROM_MARKED_PIXELS_SIZE                     MEMBER_SIZE(swift_spot_module_eeprom_data_t, markedPixels)

#define  ADS6401_EEPROM_MODULE_INFO_OFFSET                   OFFSET(swift_spot_module_eeprom_data_t, moduleInfo)
#define  ADS6401_EEPROM_MODULE_INFO_SIZE                     MEMBER_SIZE(swift_spot_module_eeprom_data_t, moduleInfo)

#define  ADS6401_EEPROM_WALK_ERROR_OFFSET                    OFFSET(swift_spot_module_eeprom_data_t, WalkError)
#define  ADS6401_EEPROM_WALK_ERROR_SIZE                      MEMBER_SIZE(swift_spot_module_eeprom_data_t, WalkError)

#define  ADS6401_EEPROM_SPOT_ENERGY_OFFSET                   OFFSET(swift_spot_module_eeprom_data_t, SpotEnergy)
#define  ADS6401_EEPROM_SPOT_ENERGY_SIZE                     MEMBER_SIZE(swift_spot_module_eeprom_data_t, SpotEnergy)

#define  ADS6401_EEPROM_RAW_DEPTH_MEAN_OFFSET                   OFFSET(swift_spot_module_eeprom_data_t, RawDepthMean)
#define  ADS6401_EEPROM_RAW_DEPTH_MEAN_SIZE                     MEMBER_SIZE(swift_spot_module_eeprom_data_t, RawDepthMean)

#define  ADS6401_EEPROM_NOISE_OFFSET                      OFFSET(swift_spot_module_eeprom_data_t, noise)
#define  ADS6401_EEPROM_NOISE_SIZE                        MEMBER_SIZE(swift_spot_module_eeprom_data_t, noise)

#define  ADS6401_EEPROM_CHKSUM_OFFSET                      OFFSET(swift_spot_module_eeprom_data_t, checksum)
#define  ADS6401_EEPROM_CHKSUM_SIZE                        MEMBER_SIZE(swift_spot_module_eeprom_data_t, checksum)

// -------------------- swift SPOT module definition end -------------

// -------------------- swift FLOOD module definition start -------------

// EEPROM-I2C--P24C256F-D4H-MIR
#define FLOOD_MODULE_EEPROM_CAPACITY_SIZE                   (32*1024)  // 32*1024, unit is bytes
#define FLOOD_MODULE_EEPROM_PAGE_SIZE                       64

#define FLOOD_MODULE_SWIFT_OFFSET_SIZE                      1920  //960 * 2

#define FLOOD_MODULE_CRC32_SPACE                            4
#define FLOOD_MODULE_OFFSET_NUM                             8

//page0
#define FLOOD_MODULE_DEVICE_VERSION_LENGTH                  6
#define FLOOD_MODULE_DEVICE_SN_LENGTH                       16
#define FLOOD_MODULE_PAGE0_RESERVED_SIZE                    (FLOOD_MODULE_EEPROM_PAGE_SIZE - FLOOD_MODULE_DEVICE_VERSION_LENGTH - FLOOD_MODULE_DEVICE_SN_LENGTH - FLOOD_MODULE_CRC32_SPACE)

//page1
#define FLOOD_MODULE_DEVICE_MODULEINFO_LENGTH               60

//page2

//sram data
#define FLOOD_MODULE_SRAM_ZONE_OCUPPY_SPACE                 512
#define FLOOD_MODULE_SRAM_ZONE_VALID_DATA_LENGTH            480

//offset
#define FLOOD_MODULE_OFFSET_VALID_DATA_LENGTH               960

// from SpadisPC\SpadisLib\eepromSettings.h
enum SwiftFloodEEPROMSizeInfo
{
    PageStartSoftwareVersion = 0,
    PageLenSoftwareVersion = 1,
    PageStartModuleInfo = 1,
    PageLenModuleInfo = 1,
    PageStartCalibrationData = 2,
    PageLenCalibrationData = 1,
    PageStartSram1Data = 3,
    PageLenSram1Data = 32,
    PageStartSram0Data = 35,
    PageLenSram0Data = 32,
    PageStartOffset1Data = 67,
    PageLenOffset1Data = 60,
    PageStartOffset0Data = 127,
    PageLenOffset0Data = 60,
    PageStartOffsetCRC = 187,
    PageLenOffsetCRC = 1
};


#pragma pack(1)
typedef struct SwiftFloodModuleEepromData
{
    // In Swift EEPROM, one page has 64 bytes.
    // Page 0
    char            Version[FLOOD_MODULE_DEVICE_VERSION_LENGTH];//6
    char            SerialNumber[FLOOD_MODULE_DEVICE_SN_LENGTH];//16
    unsigned int    Crc32Pg0;//4
    unsigned char   Pg0Reserved[FLOOD_MODULE_PAGE0_RESERVED_SIZE];//38

    //page 1
    char            ModuleInfo[FLOOD_MODULE_DEVICE_MODULEINFO_LENGTH];//60
    unsigned int    Crc32Pg1; //4

    //page2
    unsigned char   tdcDelay[2];
    float           intrinsic[9]; // 36 bytes
    float           indoorCalibTemperature; // Calibration temperature.
    float           outdoorCalibTemperature; // Calibration temperature.
    float           indoorCalibRefDistance; // Calibration reference distance.
    float           outdoorCalibRefDistance; // Calibration reference distance.
    unsigned int    Crc32Pg2;
    unsigned char   Pg2Reserved[FLOOD_MODULE_EEPROM_PAGE_SIZE-2-36-16-4]; //64-2-36-16-4=6

    //page 3---66
    unsigned char   sramData[ALL_ROISRAM_GROUP_SIZE];// 512*8 ,BUT 480(datat)+4(crc) is valid for every 512 bytes

    //page 67-186
    float           spotOffset[FLOOD_MODULE_SWIFT_OFFSET_SIZE]; // 64*60=3840 bytes = 960*4, 60 pages  for one offset.  here we have 2 offset

    //page 187
    unsigned int    Crc32Offset[FLOOD_MODULE_OFFSET_NUM];
    unsigned char   Pg187Reserved[FLOOD_MODULE_EEPROM_PAGE_SIZE-32]; //64-32=32
}swift_flood_module_eeprom_data_t;
#pragma pack()

#define  FLOOD_EEPROM_VERSION_INFO_OFFSET                  OFFSET(swift_flood_module_eeprom_data_t, Version)
#define  FLOOD_EEPROM_VERSION_INFO_SIZE                    MEMBER_SIZE(swift_flood_module_eeprom_data_t, Version)

#define  FLOOD_EEPROM_SN_INFO_OFFSET                       OFFSET(swift_flood_module_eeprom_data_t, SerialNumber)
#define  FLOOD_EEPROM_SN_INFO_SIZE                         MEMBER_SIZE(swift_flood_module_eeprom_data_t, SerialNumber)

#define  FLOOD_EEPROM_MODULE_INFO_OFFSET                   OFFSET(swift_flood_module_eeprom_data_t, ModuleInfo)
#define  FLOOD_EEPROM_MODULE_INFO_SIZE                     MEMBER_SIZE(swift_flood_module_eeprom_data_t, ModuleInfo)


// please refer to data struct 'swift_eeprom_data_t' in SpadisPC
#define  FLOOD_EEPROM_ROISRAM_DATA_OFFSET                  OFFSET(swift_flood_module_eeprom_data_t, sramData)
#define  FLOOD_EEPROM_ROISRAM_DATA_SIZE                    MEMBER_SIZE(swift_flood_module_eeprom_data_t, sramData)

#define  FLOOD_EEPROM_INTRINSIC_OFFSET                     OFFSET(swift_flood_module_eeprom_data_t, intrinsic)              /// 4160 0x1040
#define  FLOOD_EEPROM_INTRINSIC_SIZE                       MEMBER_SIZE(swift_flood_module_eeprom_data_t, intrinsic)         /// 9xsizeof(float)

//#define  FLOOD_EEPROM_ACCURATESPODPOS_OFFSET       OFFSET(swift_flood_module_eeprom_data_t, spotPos)
//#define  FLOOD_EEPROM_ACCURATESPODPOS_SIZE         MEMBER_SIZE(swift_flood_module_eeprom_data_t, spotPos)

#define  FLOOD_EEPROM_SPOTOFFSET_OFFSET                    OFFSET(swift_flood_module_eeprom_data_t, spotOffset)
#define  FLOOD_EEPROM_SPOTOFFSET_SIZE                      MEMBER_SIZE(swift_flood_module_eeprom_data_t, spotOffset)

#define  FLOOD_EEPROM_TDCDELAY_OFFSET                      OFFSET(swift_flood_module_eeprom_data_t, tdcDelay)
#define  FLOOD_EEPROM_TDCDELAY_SIZE                        MEMBER_SIZE(swift_flood_module_eeprom_data_t, tdcDelay)

#define  FLOOD_EEPROM_INDOOR_CALIBTEMPERATURE_OFFSET       OFFSET(swift_flood_module_eeprom_data_t, indoorCalibTemperature)
#define  FLOOD_EEPROM_INDOOR_CALIBTEMPERATURE_SIZE         MEMBER_SIZE(swift_flood_module_eeprom_data_t, indoorCalibTemperature)

#define  FLOOD_EEPROM_INDOOR_CALIBREFDISTANCE_OFFSET       OFFSET(swift_flood_module_eeprom_data_t, indoorCalibRefDistance)
#define  FLOOD_EEPROM_INDOOR_CALIBREFDISTANCE_SIZE         MEMBER_SIZE(swift_flood_module_eeprom_data_t, indoorCalibRefDistance)

#define  FLOOD_EEPROM_OUTDOOR_CALIBTEMPERATURE_OFFSET      OFFSET(swift_flood_module_eeprom_data_t, outdoorCalibTemperature)
#define  FLOOD_EEPROM_OUTDOOR_CALIBTEMPERATURE_SIZE        MEMBER_SIZE(swift_flood_module_eeprom_data_t, outdoorCalibTemperature)

#define  FLOOD_EEPROM_OUTDOOR_CALIBREFDISTANCE_OFFSET      OFFSET(swift_flood_module_eeprom_data_t, outdoorCalibRefDistance)
#define  FLOOD_EEPROM_OUTDOOR_CALIBREFDISTANCE_SIZE        MEMBER_SIZE(swift_flood_module_eeprom_data_t, outdoorCalibRefDistance)

#define FLOOD_ONE_SPOD_OFFSET_BYTE_SIZE                    3840//240*4*4

// -------------------- swift FLOOD module definition end -------------

struct adaps_dtof_intial_param {
    AdapsEnvironmentType env_type;
    AdapsMeasurementType measure_type;
    AdapsPowerMode  power_mode;
    AdapsFramerateType framerate_type;
    AdapsVcselZoneCountType vcselzonecount_type;

    UINT8 rowOffset;
    UINT8 colOffset;
    UINT8 rowSearchingRange;
    UINT8 colSearchingRange;

    // The following config are for Advanced user only, just set them to 0 (the default setting will be used in ads6401.c driver code) if you are not clear what they are.
    UINT8 grayExposure;
    UINT8 coarseExposure;
    UINT8 fineExposure;
    UINT8 laserExposurePeriod;  // laser_exposure_period, register configure value
    bool roi_sram_rolling;
};

struct adaps_dtof_runtime_param{
   AdapsEnvironmentType env_type;
   AdapsMeasurementType measure_type;
   AdapsVcselMode vcsel_mode;
   bool env_valid;
   bool measure_valid;
   bool vcsel_valid;
};

struct adaps_dtof_exposure_param{
    __u8 ptm_coarse_exposure_value;//ptm_coarse_exposure_value, register configure value
    __u8 ptm_fine_exposure_value;// ptm_fine_exposure_value, register configure value
    __u8 pcm_gray_exposure_value;// pcm_gray_exposure_value, register configure value
    __u8 exposure_period;  // laser_exposure_period, register configure value
};

struct adaps_dtof_runtime_status_param {
    bool test_pattern_enabled;
    __u32 inside_temperature_x100; //since kernel doesn't use float type, this is a expanded integer value (x100), Eg 4515 means 45.15 degree
    __u32 expected_vop_abs_x100;
    __u32 expected_pvdd_x100;
};

struct adaps_dtof_module_static_data{
    __u32 module_type;            // refer to ADS6401_MODULE_SPOT/ADS6401_MODULE_FLOOD/... of adaps_types.h file
    __u32 eeprom_capacity;       // unit is byte
    __u16 otp_vbe25;
    __u16 otp_vbd;        // unit is 10mv, or the related V X 100
    __u16 otp_adc_vref;
    __u8 chip_product_id[SWIFT_PRODUCT_ID_SIZE];
    __u8 sensor_drv_version[FW_VERSION_LENGTH];
    __u8 ready;
    __u8 eeprom_crc_matched;
};

struct adaps_dtof_update_eeprom_data{
    __u32 module_type;            // refer to ADS6401_MODULE_SPOT/ADS6401_MODULE_FLOOD/... of adaps_types.h file
    __u32 eeprom_capacity;       // unit is byte
    __u32 offset;             //eeprom data start offset
    __u32 length;                //eeprom data length
};

typedef struct {
    __u8 work_mode;
    __u16 sensor_reg_setting_cnt;
    __u16 vcsel_reg_setting_cnt;
} external_config_script_param_t;

typedef struct {
    __u32 roi_sram_size;
} external_roisram_data_size_t;

#define ADAPS_SET_DTOF_INITIAL_PARAM       \
    _IOW('T', ADAPS_DTOF_PRIVATE + 0, struct adaps_dtof_intial_param)

// This command has been deprecated; do not use it anymore. 
#define ADAPS_UPDATE_DTOF_RUNTIME_PARAM       \
    _IOW('T', ADAPS_DTOF_PRIVATE + 1, struct adaps_dtof_runtime_param)

#define ADAPS_GET_DTOF_RUNTIME_STATUS_PARAM       \
    _IOR('T', ADAPS_DTOF_PRIVATE + 2, struct adaps_dtof_runtime_status_param)

#define ADAPS_GET_DTOF_MODULE_STATIC_DATA       \
    _IOR('T', ADAPS_DTOF_PRIVATE + 3, struct adaps_dtof_module_static_data)

#define ADAPS_GET_DTOF_EXPOSURE_PARAM       \
    _IOR('T', ADAPS_DTOF_PRIVATE + 4, struct adaps_dtof_exposure_param)

#define ADTOF_SET_DEVICE_REGISTER       \
    _IOW('T', ADAPS_DTOF_PRIVATE + 5, register_op_data_t *)

#define ADTOF_GET_DEVICE_REGISTER       \
    _IOR('T', ADAPS_DTOF_PRIVATE + 6, register_op_data_t *)

#define ADTOF_SET_EXTERNAL_CONFIG_SCRIPT      \
    _IOW('T', ADAPS_DTOF_PRIVATE + 7, external_config_script_param_t *)

// This command carries the risk of damaging the module calibration data and is restricted to internal use at adaps company only.
#define ADTOF_UPDATE_EEPROM_DATA       \
    _IOW('T', ADAPS_DTOF_PRIVATE + 8, struct adaps_dtof_update_eeprom_data)

#define ADTOF_SET_EXTERNAL_ROISRAM_DATA_SIZE      \
    _IOW('T', ADAPS_DTOF_PRIVATE + 9, external_roisram_data_size_t *)


#endif // FOR ADAPS_SWIFT
#endif


#endif /* _ADAPS_DTOF_UAPI_H */

