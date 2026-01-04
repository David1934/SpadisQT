#ifndef ADAPS_DTOF_H
#define ADAPS_DTOF_H

#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <linux/media.h>
#include <sys/stat.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <errno.h>

#include "misc_device.h"
#include "depthmapwrapper.h"

#define COLOR_MAP_SYNCED_WITH_PC_SPADISAPP

#define BLANK_CHAR                              '.'
#define OUTPUT_WIDTH_4_DTOF_SENSOR              210
#define OUTPUT_HEIGHT_4_DTOF_SENSOR             160

/* https://developer.android.com/reference/android/graphics/ImageFormat#DEPTH16
*
* Android dense depth image format.
* Each pixel is 16 bits, representing a depth ranging measurement from a depth camera or similar sensor. The 16-bit sample consists of a confidence value and the actual ranging measurement.
* The confidence value is an estimate of correctness for this sample. It is encoded in the 3 most significant bits of the sample, with a value of 0 representing 100% confidence, a value of 1
* representing 0% confidence, a value of 2 representing 1/7, a value of 3 representing 2/7, and so on.
*
*/

// BUT, to extend the maximum measurement distance from 8.192 meters to 16.384 meters, 
// we have modified depth16 format. The lower 14 bits represent the actual distance, and the higher 2 bits represent the confidence level.
const int CONFIDENCE_MASK = 0x3;
const int DEPTH_MASK = 0x3FFF; // For depth16 format, the low 14 bits is distance
const int DEPTH_BITS = 14;

const int COLOR_HIGH = 9000;
const int COLOR_MEDIUM = 9000;
const int COLOR_MAP_HIGH = COLOR_MEDIUM;

const int RANGE_MIN = 30;
const int RANGE_MAX = 8192;

#define MAX_DEPTH_OUTPUT_FORMATS 1
#define CHIP_TEMPERATURE_MIN_THRESHOLD             15.0
#define CHIP_TEMPERATURE_MAX_THRESHOLD             90.0

/* Minimum and maximum macros */
#define MAX(a,b)  (((a) > (b)) ? (a) : (b))
#define MIN(a,b)  (((a) < (b)) ? (a) : (b))

enum depth_confidence_level{
    ANDROID_CONF_HIGH = 0,
    ANDROID_CONF_LOW = 1,
    ANDROID_CONF_MEDIUM = 3
};

struct BGRColor
{
    u8 Blue;
    u8 Green;
    u8 Red;
};

// Frame loss checking state structure
typedef struct {
    unsigned char last_id;      // ID of the last frame
    int first_frame;            // Flag indicating first frame (initial state)
    unsigned int total_frames;  // Total frame count
    unsigned int dropped_frames;// Count of dropped frames
} FrameLossChecker;

class ADAPS_DTOF
{

public:
    ADAPS_DTOF(struct sensor_params params);
    ~ADAPS_DTOF();

    int adaps_dtof_initilize();
    void GetDepth4watchSpot(const u16 depth16_buffer[], const int outImgWidth, u8 x, u8 y, u16 *distance, u8 *confidence);
    void ConvertDepthToColoredMap(const u16 depth16_buffer[], u8 depth_colored_map[], u8 depth_confidence_map[], const int outImgWidth, const int outImgHeight);
    void ConvertGreyscaleToColoredMap(u16 depth16_buffer[], u8 depth_colored_map[], int outImgWidth, int outImgHeight);
    int dtof_frame_decode(unsigned int frm_sequence, unsigned char *frm_rawdata, int frm_rawdata_size, u16 depth16_buffer[], pc_pkt_t *point_cloud_buffer, enum sensor_workmode swk);
    void adaps_dtof_release();
    int dumpSpotCount(const u16 depth16_buffer[], const int outImgWidth, const int outImgHeight, const uint32_t frm_sequence, const uint32_t out_frame_cnt, int decodeRet, int callline);
    int depthMapDump(const u16 depth16_buffer[], const int outImgWidth, const int outImgHeight, const uint32_t out_frame_cnt, int callline);
    int dump_frame_headinfo(unsigned int frm_sequence, unsigned char *frm_rawdata, int frm_rawdata_size, enum sensor_workmode swk);
    SpotPoint* get_spcific_histogram(uint16_t x, uint16_t y);

private:
    Misc_Device *p_misc_device;

    struct sensor_params m_sns_param;
    uint64_t m_exposure_time;
    int32_t  m_sensitivity;
    void* p_eeprominfo;
    SetWrapperParam set_param;

    struct BGRColor m_basic_colors[5];
    u16 m_LimitedMaxDistance;
    u16 m_rangeHigh;
    u16 m_rangeLow;

    void *m_handlerDepthLib;
    char m_DepthLibversion[32];
    char m_DepthLibConfigXmlPath[128];
    bool m_conversionLibInited;
    uint32_t m_input_frame_cnt;
    uint32_t m_output_frame_cnt;
    WrapperDepthOutput depthOutputs[MAX_DEPTH_OUTPUT_FORMATS];
    WrapperDepthInput depthInput;
    WrapperDepthCamConfig depthConfig;
    FrameLossChecker flc;
    void* loaded_roi_sram_data;
    uint8_t* copied_roisram_4_anchorX;
    uint32_t loaded_roi_sram_size;
    int sub_frame_cnt_per_image_frame;
    volatile bool skip_frame_decode;

    u8 frameCoordinatesMap[OUTPUT_HEIGHT_4_DTOF_SENSOR][OUTPUT_WIDTH_4_DTOF_SENSOR];

    void roisram_anchor_preproccess(uint8_t *roisram_buf, uint32_t roisram_buf_size);
    int FillSetWrapperParamFromEepromInfo(uint8_t* pEEPROMData, SetWrapperParam* setparam, WrapperDepthInitInputParams* initInputParams);
    int initParams(WrapperDepthInitInputParams* initInputParams, WrapperDepthInitOutputParams* initOutputParams);
    void PrepareFrameParam(WrapperDepthCamConfig *wrapper_depth_map_config);
    u8 normalizeGreyscale(u16 range);
    void Distance_2_BGRColor(int bucketNum, float bucketSize, u16 distance, struct BGRColor *destColor);
    int roiCoordinatesDumpCheck(uint8_t* roi_sram_data, int outImgWidth, int outImgHeight, int roisram_group_index);
    int multipleRoiCoordinatesDumpCheck(uint8_t* multiple_roi_sram_data, u16 length, int outImgWidth, int outImgHeight);
    void init_frame_checker(FrameLossChecker *checker);
    int check_frame_loss(FrameLossChecker *checker, const unsigned char *buffer, size_t buffer_size);
    float get_frame_loss_rate(const FrameLossChecker *checker);
};

#endif // ADAPS_DTOF_H

