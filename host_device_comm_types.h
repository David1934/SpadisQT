#ifndef HOST_DEVICE_COMM_TYPES_H
#define HOST_DEVICE_COMM_TYPES_H

#include <sys/time.h>

#define SCRIPT_FILE_SIZE                        (4 * 1024)  //4K
#define ERROR_MSG_MAX_LENGTH                    128
#define PER_ROI_SRAM_MAX_SIZE                   512     //unit is bytes, actual use size is 480 bytes, the remaining 32 bytes is 0
#define ZONE_COUNT_PER_SRAM_GROUP               4
#define MAX_CALIB_SRAM_ROTATION_GROUP_CNT       9
#define SWIFT_PRODUCT_ID_SIZE                   12
#define FW_VERSION_LENGTH                       12

#if !defined(BOOLEAN)
//#define BOOLEAN                                 unsigned char
typedef unsigned char                           BOOLEAN;
#endif

#if !defined(BIT)
#define BIT(n)                                  (1 << n)
#endif

//The spadis app running on the PC corresponding to the client,
//and the application service running on the rk3568 corresponding to the server
typedef enum
{
    // from PC to device side
    CMD_HOST_SIDE_SET_COLORMAP_RANGE_PARAM          = 0x0001,
    CMD_HOST_SIDE_GET_MODULE_STATIC_DATA            = 0x0002, // Host PC side sends a command to get the calibration data from the EEPROM and otp
    CMD_HOST_SIDE_SET_ROI_SRAM_DATA                 = 0x0003,
    CMD_HOST_SIDE_START_CAPTURE                     = 0x0004, // Host PC side sends a command to start data capture on the device side
    CMD_HOST_SIDE_STOP_CAPTURE                      = 0x0005, // Host PC side sends a command to stop data capture on the device side
    CMD_HOST_SIDE_SET_SENSOR_REGISTER               = 0x0006,
    CMD_HOST_SIDE_GET_SENSOR_REGISTER               = 0x0007,
    CMD_HOST_SIDE_SET_VCSLDRV_OP7020_REGISTER       = 0x0008,
    CMD_HOST_SIDE_GET_VCSLDRV_OP7020_REGISTER       = 0x0009,
    CMD_HOST_SIDE_SET_SPOT_WALKERROR_DATA           = 0x000A,
    CMD_HOST_SIDE_SET_SPOT_OFFSET_DATA              = 0x000B,
    CMD_HOST_SIDE_SET_WALKERROR_ENABLE              = 0x000C,
    CMD_HOST_SIDE_SET_EEPROM_DATA                   = 0x000D,
    CMD_HOST_SIDE_SET_DEVICE_REBOOT                 = 0x000E,
    CMD_HOST_SIDE_SET_RTC_TIME                      = 0x000F,
    CMD_HOST_SIDE_SET_EXPOSURE_TIME                 = 0x0010,

    // from device side to PC side
    CMD_DEVICE_SIDE_REPORT_MODULE_STATIC_DATA       = 0x1000, // Device side sends the static_module data as requested by the client's CMD_HOST_SIDE_GET_MODULE_STATIC_DATA command
    CMD_DEVICE_SIDE_REPORT_FRAME_RAW_DATA           = 0x1001, // Device side sends the raw data as requested by the client's start capture command
    CMD_DEVICE_SIDE_REPORT_SENSOR_REGISTER          = 0x1002,
    CMD_DEVICE_SIDE_REPORT_VCSLDRV_OP7020_REGISTER  = 0x1003,
    CMD_DEVICE_SIDE_REPORT_STATUS                   = 0x1004,
    CMD_DEVICE_SIDE_REPORT_FRAME_DEPTH16_DATA       = 0x1005, // Device side sends the raw data as requested by the client's start capture command
    CMD_DEVICE_SIDE_REPORT_FRAME_POINTCLOUD_DATA    = 0x1006, // Device side sends the raw data as requested by the client's start capture command
} cmdType_t;

typedef enum
{
    // from device side to PC side
    CMD_DEVICE_SIDE_NO_ERROR                        = 0x0000,
    CMD_DEVICE_SIDE_ERROR_INVALID_PARAM             = 0x0001, // if there is some invalid input parameters
    CMD_DEVICE_SIDE_ERROR_MISMATCHED_WORK_MODE      = 0x0002, // when work_mode and the register config in script buffer is mismatched.
    CMD_DEVICE_SIDE_ERROR_WRITE_REGISTER            = 0x0003,
    CMD_DEVICE_SIDE_ERROR_READ_REGISTER             = 0x0004,
    CMD_DEVICE_SIDE_ERROR_CAPTURE_ABORT             = 0x0005,
    CMD_DEVICE_SIDE_ERROR_INVALID_ROI_SRAM_SIZE     = 0x0006,
    CMD_DEVICE_SIDE_ERROR_FAIL_TO_START_CAPTURE     = 0x0007,
    CMD_DEVICE_SIDE_ERROR_INVALID_WALKERROR_SIZE    = 0x0008,
    CMD_DEVICE_SIDE_ERROR_INVALID_SPOTOFFSET_SIZE   = 0x0009,
    CMD_DEVICE_SIDE_ERROR_FAIL_TO_UPDATE_EEPROM     = 0x000A,
    CMD_DEVICE_SIDE_ERROR_INVALID_EEPROM_UPD_PARAM  = 0x000B,
    CMD_DEVICE_SIDE_ERROR_INVALID_REBOOT_REASON     = 0x000C,
} error_code_t;

typedef enum
{
    // from PC to device side
    CMD_HOST_SIDE_REBOOT_NO_REASON                  = 0x0000, // Don't allow set to this one if PC side want to reboot device really.
    CMD_HOST_SIDE_REBOOT_FOR_EEPROM_UPDATE_DONE     = 0x0001,
} device_reboot_reason_t;

#pragma pack(1)

typedef struct CommandData_s
{
    UINT16  cmd;   // command type
    char    param[0];
} CommandData_t;

typedef enum swift_data_type {
    MODULE_STATIC_DATA          = BIT(0),
    FRAME_RAW_DATA              = BIT(1),
    FRAME_DECODED_DEPTH16       = BIT(2),   // for PCM and PTM mode (FHR, PHR)
    FRAME_DECODED_POINT_CLOUD   = BIT(3),
    FRAME_DECODED_DEPTH16_XY    = BIT(4),
} swift_datatype_t;

enum {
    ADS6401_ROI_REG_F7 = 0,
    ADS6401_ROI_REG_F8,
    ADS6401_ROI_REG_F9,
    ADS6401_ROI_REG_FA,

    ADS6401_ROI_REG_FB,
    ADS6401_ROI_REG_FC,
    ADS6401_ROI_REG_FD,
    ADS6401_ROI_REG_FE,
};

typedef struct exposure_time_param_s
{
    UINT8 grayExposure;
    UINT8 coarseExposure;
    UINT8 fineExposure;
} exposure_time_param_t;

typedef struct rtc_time_param_s
{
    struct timeval strUtcTime;
    struct timezone strTimeZone;
} rtc_time_param_t;

typedef struct colormap_range_param
{
    int GrayScaleMinMappedRange;
    int GrayScaleMaxMappedRange;
    float RealDistanceMinMappedRange;
    float RealDistanceMaxMappedRange;
} colormap_range_param_t;

typedef struct walkerror_enable_param
{
    BOOLEAN                 walkerror_enable;
} walkerror_enable_param_t;

typedef struct roisram_data_param
{
    BOOLEAN                 roi_sram_rotate;
    UINT32                  roisram_data_size;      // roi sram buffer size to be loaded, it should be an integer multiple of (PER_ROI_SRAM_MAX_SIZE * ZONE_COUNT_PER_SRAM_GROUP) or 0, 
                                                    // it shoud <= (PER_ROI_SRAM_MAX_SIZE * ZONE_COUNT_PER_SRAM_GROUP * MAX_CALIB_SRAM_ROTATION_GROUP_CNT) 
    CHAR                    roisram_data[0];        // loaded roi sram buffer, No this member if roisram_data_size == 0
} roisram_data_param_t;

typedef struct spot_walkerror_data_param
{
    UINT32                  walkerror_data_size;
    CHAR                    walkerror_data[0];        // loaded walkerror data buffer, No this member if walkerror_data_size == 0
} spot_walkerror_data_param_t;

typedef struct spot_offset_data_param
{
    UINT32                  offset_data_size;
    CHAR                    offset_data[0];        // loaded offset data buffer, No this member if offset_data_size == 0
} spot_offset_data_param_t;

typedef struct eeprom_data_update_param
{
    UINT32                  offset;             //eeprom data start offset
    UINT32                  length;                //eeprom data length
    CHAR                    eeprom_data[0];        // eeprom data buffer to be updated, No this member if length == 0
} eeprom_data_update_param_t;

typedef struct capture_req_param
{
    UINT8                   work_mode;              // refer to swift_workmode_t of adaps_types.h
    UINT8                   env_type;               // refer to AdapsEnvironmentType of adaps_types.h
    UINT8                   measure_type;           // refer to AdapsMeasurementType of adaps_types.h
    UINT8                   framerate_type;         // refer to AdapsFramerateType of adaps_types.h
    UINT8                   power_mode;             // refer to AdapsPowerMode of adaps_types.h
    UINT8                   walkerror_enable;
    UINT8                   req_out_data_type;      // refer to enum swift_data_type of this .h file
    BOOLEAN                 compose_subframe;
    BOOLEAN                 expand_pixel;
    BOOLEAN                 mirror_x;
    BOOLEAN                 mirror_y;
    BOOLEAN                 laserEnable;
    BOOLEAN                 vopAdjustEnable;
    UINT8                   rowSearchingRange;
    UINT8                   colSearchingRange;
    UINT8                   rowOffset;
    UINT8                   colOffset;
    exposure_time_param_t   expose_param;
    BOOLEAN                 script_loaded;
    UINT32                  script_size;            // set to 0 if script_loaded == false
    CHAR                    script_buffer[0];       // file content from the script file, No this member if script_loaded == false
} capture_req_param_t;

typedef struct frame_buffer_param_s
{
    UINT8                   data_type;                  // refer to enum swift_data_type of this .h file
    UINT8                   work_mode;                  // refer to enum adaps_work_mode of adaps_types.h
    UINT16                  frm_width;                  // 
    UINT16                  frm_height;                 // 
    UINT16                  padding_bytes_per_line;     // for PCM/FHR/PHR raw data, rockchip 4352 - 4104 for FHR
    UINT8                   env_type;                   // refer to AdapsEnvironmentType of adaps_types.h
    UINT8                   measure_type;               // refer to AdapsMeasurementType of adaps_types.h
    UINT8                   framerate_type;             // refer to AdapsFramerateType of adaps_types.h
    UINT8                   power_mode;                 // refer to AdapsPowerMode of adaps_types.h
    UINT32                  curr_pvdd;                  // the integer part of the PVDD voltage multiplied by 100
    UINT32                  curr_vop_abs;               // the integer part of the absolute value of the VOP voltage multiplied by 100
    UINT32                  curr_inside_temperature;    // the integer part of the current temperature (in degrees Celsius) multiplied by 100.
    UINT8                   ptm_coarse_exposure_value;  //ptm_coarse_exposure_value, register configure value
    UINT8                   ptm_fine_exposure_value;    // ptm_fine_exposure_value, register configure value
    UINT8                   pcm_gray_exposure_value;    // pcm_gray_exposure_value, register configure value
    UINT8                   exposure_period;            // laser_exposure_period, register configure value
    UINT64                  frame_sequence;
    UINT64                  frame_timestamp_us;
    UINT16                  mipi_rx_fps;
    UINT8                   roi_data_index;
    UINT32                  buffer_size;                // the size for the following buffer, unit is byte
    CHAR                    buffer[0];
} frame_buffer_param_t;

typedef struct module_static_data_s
{
    UINT8                   data_type;              // refer to enum swift_data_type of this .h file
    UINT32                  module_type;            // refer to ADS6401_MODULE_SPOT and ADS6401_MODULE_FLOOD of adaps_types.h file
    UINT32                  eeprom_capacity;        // unit is byte
    UINT16                  otp_vbe25;
    UINT16                  otp_vbd;        // unit is 10mv, or the related V X 100
    UINT16                  otp_adc_vref;
    CHAR                    chip_product_id[SWIFT_PRODUCT_ID_SIZE];  // read out from ads6401 e-fuse data
    CHAR                    sensor_drv_version[FW_VERSION_LENGTH];
    CHAR                    algo_lib_version[FW_VERSION_LENGTH];
    CHAR                    sender_lib_version[FW_VERSION_LENGTH];
    CHAR                    spadisQT_version[FW_VERSION_LENGTH];
    UINT32                  eeprom_data_size;        // unit is byte
    CHAR                    eeprom_data[0];
} module_static_data_t;

typedef struct error_report_param_s
{
    UINT16                  err_code;
    UINT16                  responsed_cmd;
    CHAR                    err_msg[ERROR_MSG_MAX_LENGTH];
} error_report_param_t;

typedef struct device_reboot_request_s
{
    UINT8                   reboot_reason_code;
    CHAR                    reboot_reason_msg[ERROR_MSG_MAX_LENGTH];
} device_reboot_request_t;

#ifndef register_op_data_type
#define register_op_data_type

    enum register_op_type
    {
        reg_op_type_read = 0,
        reg_op_type_write,
    };
    
    enum register_op_width_type
    {
        reg8_data8_type = 0,        // both the reigster address and value's width are 8 bits
        reg16_data8_type,           // the reigster address is 16 bits width and its value is 8 bits width
        reg16_data16_type,          // both the reigster address and value's width are 16 bits
        reg8_data16_type            // the reigster address is 8 bits width and its value is 16 bits width
    };
    
    typedef struct {
        UINT8 i2c_address;
        UINT8 reg_op_type;          // refer to enum register_op_type definition
        UINT8 reg_op_width_type;    // refer to enum register_op_width_type definition
        UINT16 reg_addr;            // only use low 8 bits when register address width is 8 bits
        UINT16 reg_val;             // only use low 8 bits when register value width is 8 bits
    } register_op_data_t;
#endif // register_op_data_type

#pragma pack()


#endif // HOST_DEVICE_COMM_TYPES_H

