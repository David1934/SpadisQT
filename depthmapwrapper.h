#ifndef __DEPTHMAP_WRAPPER_H__
#define __DEPTHMAP_WRAPPER_H__



// ***** start to move some definition to here to let SpadisQT build pass ****
#include "adaps_types.h"

//==============================================================================
// MACROS
//==============================================================================
#ifdef _MSC_VER
#define CP_DLL_PUBLIC __declspec(dllexport)
#else
#define CP_DLL_PUBLIC __attribute__ ((visibility ("default")))
#endif

#if defined(ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB) // the old algo lib 3.3.2, which supports Android.
#define MAX_SRAM_DATA_NUMBERS      1
#else
#define MAX_SRAM_DATA_NUMBERS      9
#endif

#define ADAPS_SPARSE_POINT_POSITION_DATA_SIZE   960
#define AdapsAlgoLibVersionLength               32
#define ZONE_SIZE                               (4)
#define SWIFT_SPOT_COUNTS_PER_ZONE              (240)

struct AdapsSparsePointPositionData
{
    UINT32 x_pos[ADAPS_SPARSE_POINT_POSITION_DATA_SIZE * MAX_SRAM_DATA_NUMBERS];
    UINT32 y_pos[ADAPS_SPARSE_POINT_POSITION_DATA_SIZE * MAX_SRAM_DATA_NUMBERS];
    uint32_t hist[ADAPS_SPARSE_POINT_POSITION_DATA_SIZE];
};

typedef struct pc_pack {
    float X;
    float Y;
    float Z;
    float c;
} pc_pkt_t;

struct SpotPoint {
    uint16_t x;
    uint16_t y;
    uint8_t  zoneId;     // the zone of spot belong to  
    uint8_t  indexInZone;// index in the zone

    uint16_t histRaw[768];
    uint32_t histConved[768];

};

typedef enum {
    WRAPPER_CAM_FORMAT_NONE,
    WRAPPER_CAM_FORMAT_IR,
    WRAPPER_CAM_FORMAT_DEPTH16,
    WRAPPER_CAM_FORMAT_DEPTH_POINT_CLOUD,
    WRAPPER_CAM_FORMAT_DEPTH_X_Y,
    WRAPPER_CAM_FORMAT_MAX,
} WrapperDepthFormat;

typedef enum {
    DEPTH_OUT_NORMAL,       ///< No change
    DEPTH_OUT_MIRROR,       ///< Mirror(horizontal)
    DEPTH_OUT_FLIP,         ///< Flip(vertical)
    DEPTH_OUT_MIRROR_FLIP,  ///< Mirror/Flip(h/v)
} RotateConfig;

typedef struct {
    int32_t  bitsPerPixel;
    uint32_t strideBytes;
    uint32_t sliceHeight;
    size_t   planeSize;
} WrapperDepthFormatParams;

//begin: add by hzt 2021-12-6 for adaps control
typedef struct {
    uint32_t ulRoiIndex;
    uint8_t* pucSramData;
    uint32_t ulSramDataSize;
} WrapperDepthSramSpodposDataInfo;

typedef struct {
    uint8_t FocusLeftTopX; // default is 0;
    uint8_t FocusLeftTopY; // default is 0;
    uint8_t FocusRightBottomX; // default is 210;
    uint8_t FocusRightBottomY; // default is 160;
} FocusRoi;

typedef struct CircleForMask {
    float CircleMaskR;
    int   CircleMaskCenterX;
    int   CircleMaskCenterY;
}CircleForMask;

typedef struct {
    AdapsEnvironmentType env_type_in;
    AdapsMeasurementType measure_type_in;
    float laser_realtime_tempe;
    AdapsEnvironmentType *advised_env_type_out;
    AdapsMeasurementType *advised_measure_type_out;
    int32_t focutPoint[2];// 0 is x,1 is y
    WrapperDepthSramSpodposDataInfo strSpodPosData;
    FocusRoi             focusRoi;
//    bool            walkerror_enable;
} AdapsParamAndOutForProcessEveryFrame;
//end: add by hzt 2021-12-6 for adaps control

typedef enum {
    WRAPPER_EXP_MODE_ME,
    WRAPPER_EXP_MODE_AE,
    WRAPPER_EXP_MODE_NONE
} WrapperDepthExpModeType;

typedef struct {
    float min_fps;
    float max_fps;
    float video_min_fps;
    float video_max_fps;
} WrapperDepthCamFpsRange;

typedef struct {
    int64_t in_depth_map_me_val;
    int64_t in_out_depth_map_ae_val;
    int64_t in_depth_map_frame_num;
    int64_t in_out_depth_map_timestamp;
    int64_t out_depth_map_laser_strength_val;
    WrapperDepthExpModeType in_depth_map_exp_mode;
    WrapperDepthCamFpsRange in_depth_map_fps_range;
    AdapsParamAndOutForProcessEveryFrame frame_parameters;
} WrapperDepthCamConfig;

typedef struct {
    WrapperDepthFormat format;
    WrapperDepthFormatParams formatParams;
    struct AdapsSparsePointPositionData sPPData;
    union
    {
        uint8_t*  out_depth_image;
        pc_pkt_t* out_pcloud_image;
    };
    int32_t out_image_fd;
    union
    {
        uint32_t  out_image_length;
        uint32_t* count_pt_cloud;
    };
#if !defined(ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB)
    struct SpotPoint* (*outAllPointsPtr)[ZONE_SIZE][SWIFT_SPOT_COUNTS_PER_ZONE];
#endif
} WrapperDepthOutput;

typedef struct {
    WrapperDepthFormatParams formatParams;
    const int8_t* in_image;
#if defined(ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB)
    int32_t in_image_fd;
#else
    int32_t in_image_size;
#endif
} WrapperDepthInput;

//begin: add by hzt 2021-12-6 for adaps control
typedef struct {
    uint8_t work_mode;
    bool compose_subframe;
    bool expand_pixel;
    bool walkerror; // 0: not apply walkerror , 1:apply walkerror
    AdapsMirrorFrameSet mirror_frame;
    float* adapsLensIntrinsicData;          // 9xsizeof(float)
    float* adapsSpodOffsetData;             // 4x240xsizeof(float)
#if defined(ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB)
    float* accurateSpotPosData;             // 4x240xsizeof(float)x2 delete
#else
    uint32_t adapsSpodOffsetDataLength;
#endif
    uint8_t ptm_fine_exposure_value;        // fine exposure value, 0 - 255
    uint8_t exposure_period;                // exposure_period, 0 - 255
    float cali_ref_tempe[2];  //[0] for indoor, [1] for outdoor
    float cali_ref_depth[2];  //[0] for indoor, [1] for outdoor
    AdapsEnvironmentType env_type;  // value 0-->indoor, value 1 -->outdoor
    AdapsMeasurementType measure_type; //value 0-->normal distance, 1-->short distance
    uint8_t* proximity_hist; //256 bytes for eeprom
    uint8_t roiIndex;   // Only zoom focus Camx version support the "roiIndex"
    // TODO - after v1.2.0
    uint8_t* OutAlgoVersion;  // OutAlgoVersion[AdapsAlgoVersionLength];
    uint8_t zone_cnt;
    uint8_t peak_index; // 0: double peaks   1: select first peak 
    uint8_t* spot_cali_data;//add 2023-11-7
    //zondID | spotID | X     | Y     | paramD | paramX | paramY | paramZ | param0 | dummy | dummy2
    //1byte  | 1byte  |1byte  |1byte  | 4byte  | 4byte  | 4byte  | 4byte  | 4byte  | 1byte | 1byte
    //Every spot has 26byte data (zondID to dummy2)
    uint8_t* walk_error_para_list;
#if !defined(ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB)
    uint32_t walk_error_para_list_length;
    uint8_t* calibrationInfo; // length:64 ,byte 0-8 ofilm version ,byte 9-26 adaps sdk version v4.0-180-g3b7adfgh
#ifdef WINDOWS_BUILD
    bool dump_data;
#endif
#endif
} SetWrapperParam;

typedef struct {
    const char*     configFilePath;
    int32_t         width;
    int32_t         height;
    int32_t         dm_width;
    int32_t         dm_height;
    uint8_t*        pRawData;
    uint32_t        rawDataSize;
    RotateConfig    rotateConfig;
    uint32_t        outputPlaneCount;
    uint32_t        outputPlaneFormat[WRAPPER_CAM_FORMAT_MAX];
    SetWrapperParam setparam;
} WrapperDepthInitInputParams;

typedef struct {
    uint64_t* exposure_time;
    int32_t*  sensitivity;
} WrapperDepthInitOutputParams;
// ***** end to move some definition to here to let SpadisQT build pass ****




// Class factories
#ifdef __cplusplus
extern "C" {
#endif
CP_DLL_PUBLIC
int  DepthMapWrapperCreate(
    void** handler,
    WrapperDepthInitInputParams  inputParams,
    WrapperDepthInitOutputParams outputParams);

CP_DLL_PUBLIC
bool DepthMapWrapperProcessFrame(
    void* handler,
    WrapperDepthInput in_image,
    WrapperDepthCamConfig *wrapper_depth_map_config,
    uint32_t num_outputs,
    WrapperDepthOutput outputs[]);

CP_DLL_PUBLIC
void DepthMapWrapperDestroy(void * handler);

CP_DLL_PUBLIC
void DepthMapWrapperGetVersion(char* version);

CP_DLL_PUBLIC
void DepthMapWrapperSetCircleMask(void* pDepthMapWrapper, CircleForMask circleForMask);


#ifdef __cplusplus
}
#endif

#endif //__DEPTHMAP_WRAPPER_H__

