#ifndef ADAPS_TYPES_H
#define ADAPS_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus


typedef unsigned char       BOOL;
typedef char                CHAR;
typedef unsigned char       UCHAR;
typedef int                 INT;
typedef unsigned int        UINT;
typedef float               FLOAT;

typedef unsigned char       UINT8;
typedef char                INT8;
typedef uint16_t            UINT16;
typedef int16_t             INT16;
typedef uint32_t            UINT32;
typedef int32_t             INT32;
typedef uint64_t            UINT64;
typedef int64_t             INT64;

#if !defined (TRUE)
#define TRUE                                1
#endif // !defined (TRUE)

#if !defined (FALSE)
#define FALSE                               0
#endif // !defined (FALSE)

#define MAX_REG_SETTING_COUNT               100
#define MAX_SCRIPT_ITEM_COUNT               400

// built-in EEPROM P24C512X-C4H-MIR, VcselDriver OPN7020
#define ADS6401_MODULE_SPOT                 0x6401A

// built-in EEPROM P24C256F-D4H-MIR, VcselDriver PhotonIC 5015, MCU HC32L110B6YA, 
#define ADS6401_MODULE_FLOOD                0x6401B

// built-in EEPROM P24C512X-C4H-MIR, VcselDriver OPN7020
#define ADS6401_MODULE_BIG_FOV_FLOOD        0x6401C

typedef struct ADAPS_MIRROR_FRAME_SET
{
    UINT8 mirror_x;
    UINT8 mirror_y;
}AdapsMirrorFrameSet;

// adaps tof operation mode
typedef enum swift_work_mode {
    ADAPS_PTM_PHR_MODE   = 0,
    ADAPS_PCM_MODE = 1,
    ADAPS_PTM_FHR_MODE   = 2,
    ADAPS_PTM_DEBUG_PHR_MODE = 3,
    ADAPS_PTM_DEBUG_FHR_MODE=4,
    ADAPS_MODE_MAX,
} swift_workmode_t;

typedef enum
{
    AdapsFramerateTypeUninitilized,
    AdapsFramerateType15FPS,
    AdapsFramerateType25FPS,
    AdapsFramerateType30FPS,
    AdapsFramerateType60FPS,
} AdapsFramerateType;

typedef enum
{
    AdapsFrameModeUninitilized,
    AdapsFrameModeA,
    AdapsFrameModeB,
} AdapsFrameMode;

typedef enum
{
    AdapsVcselZoneCountUninitilized,
    AdapsVcselZoneCount1,
    AdapsVcselZoneCount4 = 4,
} AdapsVcselZoneCountType;

typedef enum
{
    AdapsApplyModeUninitilized,
    AdapsApplyModeFusion,
    AdapsApplyModeFocus,
} AdapsApplyMode;


typedef enum
{
    AdapsMeasurementTypeUninitilized,
    AdapsMeasurementTypeNormal,
    AdapsMeasurementTypeShort,
    AdapsMeasurementTypeFull,
} AdapsMeasurementType;

typedef enum {
    AdapsEnvTypeUninitilized,
    AdapsEnvTypeIndoor,
    AdapsEnvTypeOutdoor,
} AdapsEnvironmentType;

typedef enum {
    AdapsPowerModeUninitilized,
    AdapsPowerModeNormal,
    AdapsPowerModeDiv2,         // Div2 is not available yet now.
    AdapsPowerModeDiv3,
} AdapsPowerMode;

typedef enum {
    AdapsVcselModeUninitilized,
    AdapsVcselModeOn,
    AdapsVcselModeOff,
} AdapsVcselMode;

struct AdapsAdvisedType
{
    UINT8 AdvisedMeasurementType;
    UINT8 AdvisedEnvironmentType;
};

typedef enum
{
    AdapsDataTypeUninitilized,
    AdapsDataTypeDepth,
    AdapsDataTypeGrayscale,
} AdapsDataType;// output data format

typedef enum {
    AdapsSensorNotChange,       // the role of tof sensor will be determined by the register configure of ads6401_sensor.xml
    AdapsSensorForceAsMaster,
    AdapsSensorForceAsSlave
} AdapsSensorForceRole;

#pragma pack(1)

struct setting_rvd {   // struct seting_register_value_delay
    UINT8  reg;
    UINT8  val;
    UINT32  delayUs;
};

struct setting_r16vd {   // struct seting_register16_value_delay
    UINT16  reg;
    UINT8   val;
    UINT32  delayUs;
};

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

#ifdef __cplusplus
}
#endif // __cplusplus


#endif 

