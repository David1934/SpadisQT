#ifndef __DEPTHMAP_WRAPPER_H__
#define __DEPTHMAP_WRAPPER_H__

#define ALGO_LIB_VERSION_MAJOR                           3
#define ALGO_LIB_VERSION_MINOR                           8
#define ALGO_LIB_VERSION_REVISION                        5

#define VERSION_HEX_VALUE(major, minor, revision)        (major << 16 | minor << 8 | revision)

#define ALGO_LIB_VERSION_CODE                            VERSION_HEX_VALUE(ALGO_LIB_VERSION_MAJOR, ALGO_LIB_VERSION_MINOR << 8, ALGO_LIB_VERSION_REVISION)

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
#define MAX_SRAM_DATA_NUMBERS      MAX_CALIB_SRAM_ROLLING_GROUP_CNT
#endif

// PAY ATTENTION: when MAX_SRAM_DATA_NUMBERS=9, outAllPointsPtr[9][4][240] of WrapperDepthOutput will need 25M+ bytes RAM, 
// please ensure RAM is enough, otherwize please comment out the build option.
//#define ENABLE_HISTOGRAM_RAW_OUTPUT
#define GET_HISTOGRAM_COUNT                     (240 * 4 * 4)   // should <= (240 * 4 * 9), 240 spots per zone, 4 zones per image-frame, 9 image-frame per group, 0 mean no request histogram output.





#define ADAPS_SPARSE_POINT_POSITION_DATA_SIZE   960
#define AdapsAlgoLibVersionLength               32
#define ZONE_SIZE                               (4)
#define SWIFT_SPOT_COUNTS_PER_ZONE              (240)

#define FHR_FULL_DISTANCE_HIST_LENGTH           768 // NEED to keep match with swift algo lib
#define FHR_HIST_LENGTH                         512 // NEED to keep match with swift algo lib
#define FHR_COMBINED_HIST_LENGTH                384 // NEED to keep match with swift algo lib

#define OUT_HISTOGRAM_LEN                       384     // 512, 实际输出大小是384
#define OUT_HISTOGRAM_TYPE                      uint16_t // uint16_t

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
    uint16_t rawHistMaxValue;
#if defined(WINDOWS_BUILD) || defined(ENABLE_HISTOGRAM_RAW_OUTPUT)  // NEED to keep match with swift algo lib
    uint16_t histRaw[FHR_FULL_DISTANCE_HIST_LENGTH];                // binning histogram, max value may be great than 255
    uint16_t histRawUnBinning[FHR_FULL_DISTANCE_HIST_LENGTH];       // unbinning historgram, max value is 252, real length is FHR_HIST_LENGTH(512)
#endif
};

struct SpotHistogram {
    uint8_t x;  // 8 bits is enough, since the outout resolution of swift is 210 x 160
    uint8_t y;

    OUT_HISTOGRAM_TYPE rawHistogram[OUT_HISTOGRAM_LEN];
};

typedef enum {
    MODEL_TYPE_SPOT,
    MODEL_TYPE_FLOOD,
    MODEL_TYPE_CAMSENSE,
    MODEL_TYPE_T,
    MODEL_TYPE_CNT,
} AdapsModelType;

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
    int   CircleMaskR;
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

#if (GET_HISTOGRAM_COUNT > (MAX_SRAM_DATA_NUMBERS *ZONE_SIZE * SWIFT_SPOT_COUNTS_PER_ZONE))
    #error "GET_HISTOGRAM_COUNT is too big, please double check its definition !!!"
#endif

typedef struct {
    WrapperDepthFormat format;
    WrapperDepthFormatParams formatParams;
    struct AdapsSparsePointPositionData sPPData;
#if ALGO_LIB_VERSION_CODE >= VERSION_HEX_VALUE(3, 5, 6)
    uint8_t*  out_depth_image;
    pc_pkt_t* out_pcloud_image;
    int32_t out_image_fd;
    uint32_t  out_image_length;
    uint32_t* count_pt_cloud;
#else
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
#endif

#if !defined(ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB)
    struct SpotPoint* (*outAllPointsPtr)[MAX_SRAM_DATA_NUMBERS][ZONE_SIZE][SWIFT_SPOT_COUNTS_PER_ZONE];
#endif
} WrapperDepthOutput;

typedef struct {
    WrapperDepthFormatParams formatParams;
    const int8_t* in_image;
#if defined(ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB)
    int32_t in_image_fd;
#else
    int32_t in_image_size;
    int* in_sram_id;            // just for offline re-playback, added @ V3.5.4 of libadaps_swift_decode.so
#endif

#if ALGO_LIB_VERSION_CODE >= VERSION_HEX_VALUE(3, 6, 2)
    bool dump_data;
    const char* save_path;
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

#if ALGO_LIB_VERSION_CODE >= VERSION_HEX_VALUE(3, 6, 5)
    int adapsAlgoModelType;   // refer to the defintion AdapsModelType
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

CP_DLL_PUBLIC
void DepthMapWrapperGetDataInfo(void* pDepthMapWrapper, uint32_t* sramId, uint32_t* zoneId);


#ifdef __cplusplus
}
#endif

#endif //__DEPTHMAP_WRAPPER_H__

