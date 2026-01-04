#if !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)

#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <semaphore.h>
#include <string.h>

#include "common.h"
#include "utils.h"
#include "globalapplication.h"
#include "host_comm.h"
#include "depthmapwrapper.h"    // to use DepthMapWrapperGetVersion()

#ifndef SENDER_LOG_FUNC
#define SENDER_LOG_FUNC         printf
#endif

#define LOG_ERROR               DBG_ERROR
#define LOG_INFO                DBG_NOTICE
#define LOG_DEBUG               DBG_NOTICE
#define LOG_WARN                DBG_NOTICE


// 静态成员变量定义（必须！）
Host_Communication* Host_Communication::instance = NULL_POINTER;

Host_Communication* Host_Communication::getInstance() {
    if (!instance) {
        instance = new Host_Communication();
    }
    return instance;
}

// 私有构造函数
Host_Communication::Host_Communication() {
    connected = false;
    reset_data();
    p_misc_device = NULL_POINTER;
    adaps_sender_init();
}

Host_Communication::~Host_Communication()
{
    if (NULL_POINTER != instance) {
        DBG_NOTICE("------communication statistics------total txRawdataFrameCnt: %ld (%d fps), txDepth16FrameCnt: %ld (%d fps), txPointCloudFrameCnt: %ld (%d fps)---\n",
            txRawdataFrameCnt, txRawdataFrame_fps, txDepth16FrameCnt, txDepth16Frame_fps, txPointCloudFrameCnt, txPointCloudFrame_fps);

        reset_data();
        sender_destroy();
        delete instance;
        instance = NULL_POINTER;
    }
}

void Host_Communication::reset_data()
{
    if (NULL_POINTER != backuped_script_buffer)
    {
        free(backuped_script_buffer);
        backuped_script_buffer = NULL_POINTER;
    }
    backuped_script_buffer_size = 0;

    if (NULL_POINTER != loaded_walkerror_data)
    {
        free(loaded_walkerror_data);
        loaded_walkerror_data = NULL_POINTER;
    }
    loaded_walkerror_data_size = 0;

    if (NULL_POINTER != loaded_spotoffset_data)
    {
        free(loaded_spotoffset_data);
        loaded_spotoffset_data = NULL_POINTER;
    }
    loaded_spotoffset_data_size = 0;

    if (NULL_POINTER != loaded_ref_distance_data)
    {
        free(loaded_ref_distance_data);
        loaded_ref_distance_data = NULL_POINTER;
    }
    
    if (NULL_POINTER != loaded_lens_intrinsic_data)
    {
        free(loaded_lens_intrinsic_data);
        loaded_lens_intrinsic_data = NULL_POINTER;
    }

    backuped_wkmode = 0;
    req_histogram_x = 0;
    req_histogram_y = 0;
    memset(&backuped_capture_req_param, 0, sizeof(capture_req_param_t));
    txRawdataFrameCnt = 0;
    txDepth16FrameCnt = 0;
    txPointCloudFrameCnt = 0;
    firstRawdataFrameTimeUsec = 0;
    firstDepth16FrameTimeUsec = 0;
    eeprom_capacity = 0;
    qApp->set_roi_sram_rolling(false);
    qApp->set_size_4_loaded_roisram(0);
}

UINT8 Host_Communication::get_req_out_data_type()
{
    return backuped_capture_req_param.req_out_data_type;
}

BOOLEAN Host_Communication::get_req_compose_subframe()
{
    return backuped_capture_req_param.compose_subframe;
}

BOOLEAN Host_Communication::get_req_expand_pixel()
{
    return backuped_capture_req_param.expand_pixel;
}

BOOLEAN Host_Communication::get_req_mirror_x()
{
    return backuped_capture_req_param.mirror_x;
}

BOOLEAN Host_Communication::get_req_mirror_y()
{
    return backuped_capture_req_param.mirror_y;
}

UINT8 Host_Communication::get_req_walkerror_enable()
{
    return backuped_capture_req_param.walkerror_enable;
}

void* Host_Communication::get_loaded_ref_distance_data()
{
    return loaded_ref_distance_data;
}

void* Host_Communication::get_loaded_lens_intrinsic_data()
{
    return loaded_lens_intrinsic_data;
}

uint16_t Host_Communication::get_req_histogram_x()
{
    return req_histogram_x;
}

uint16_t Host_Communication::get_req_histogram_y()
{
    return req_histogram_y;
}

int Host_Communication::dump_buffer_data(void* dump_buf, const char *buffer_name, int callline)
{
    int dump_size = 0;
    char temp_string[128];
    Utils *utils;

    dump_size = Utils::get_env_var_intvalue(ENV_VAR_DUMP_COMM_BUFFER_SIZE);
    if (dump_size)
    {
        utils = new Utils();
        sprintf(temp_string, "-----dump %s buffer, size: %d from Line: %d-----", buffer_name, dump_size, callline);
        utils->hexdump((unsigned char *) dump_buf, dump_size, temp_string);
        delete utils;
    }

    return 0;
}

int Host_Communication::report_status(UINT16 responsed_cmd, UINT16 status_code, char *msg, int msg_length)
{
    void *async_buf;
    CommandData_t* pCmdData = NULL_POINTER;
    error_report_param_t *pErrReportParam;
    uint32_t ulBufSize = sizeof(CommandData_t) + sizeof(error_report_param_t);

    if (false == qApp->is_capture_req_from_host())
    {
        return 0;
    }

    if (false == connected || false == sender_is_working()) {
        LOG_ERROR("It seems host is not connected yet or disconnected, connected: %d.\n", connected);
        return -1;
    }

    if (msg_length > ERROR_MSG_MAX_LENGTH)
    {
        LOG_ERROR("Error message is too long.\n");
        return -1;
    }

    async_buf = sender_async_buf_alloc(ulBufSize);
    if (NULL_POINTER == async_buf) {
        LOG_ERROR("async alloc buf fail: %u.\n", ulBufSize);
        return -1;
    }

    pCmdData = (CommandData_t*) async_buf;

    pCmdData->cmd = CMD_DEVICE_SIDE_REPORT_STATUS;
    pErrReportParam  = (error_report_param_t *) pCmdData->param;
    pErrReportParam->responsed_cmd = responsed_cmd;
    pErrReportParam->err_code = status_code;
    memcpy(pErrReportParam->err_msg, msg, msg_length);

    int result = sender_async_send_msg(async_buf, ulBufSize, 0);

    if (result < 0) {
        //sender_async_buf_free(pstrCmdData);
        LOG_ERROR("async send raw data fail: %u.\n", ulBufSize);
        return result;
    }

    LOG_DEBUG("%s(%s) done!\n", __FUNCTION__, msg);

    return 0;
}

int Host_Communication::dump_frame_param(frame_buffer_param_t *pDataBufferParam)
{
    LOG_DEBUG("UINT8            data_type = 0x%x;               // refer to enum swift_data_type of this .h file", pDataBufferParam->data_type);
    LOG_DEBUG("UINT8            work_mode = %d;                 // refer to swift_workmode_t of adaps_types.h", pDataBufferParam->work_mode);
    LOG_DEBUG("UINT16           frm_width = %d;", pDataBufferParam->frm_width);
    LOG_DEBUG("UINT16           frm_height = %d;", pDataBufferParam->frm_height);
    LOG_DEBUG("UINT16           padding_bytes_per_line = %d;    // for PCM/FHR/PHR raw data, rockchip 4352 - 4104 for FHR", pDataBufferParam->padding_bytes_per_line);
    LOG_DEBUG("UINT8            env_type = %d;                  // refer to AdapsEnvironmentType of adaps_types.h", pDataBufferParam->env_type);
    LOG_DEBUG("UINT8            measure_type = %d;              // refer to AdapsMeasurementType of adaps_types.h", pDataBufferParam->measure_type);
    LOG_DEBUG("UINT8            framerate_type = %d;            // refer to AdapsFramerateType of adaps_types.h", pDataBufferParam->framerate_type);
    LOG_DEBUG("UINT8            power_mode = %d;                // refer to AdapsPowerMode of adaps_types.h", pDataBufferParam->power_mode);

    LOG_DEBUG("UINT32           curr_pvdd = %d;                 // the integer part of the PVDD voltage multiplied by 100", pDataBufferParam->curr_pvdd);
    LOG_DEBUG("UINT32           curr_vop_abs = %d;              // the integer part of the absolute value of the VOP voltage multiplied by 100", pDataBufferParam->curr_vop_abs);
    LOG_DEBUG("UINT32           curr_inside_temperature = %d;   // the integer part of the current temperature (in degrees Celsius) multiplied by 100", pDataBufferParam->curr_inside_temperature);
    LOG_DEBUG("UINT8            ptm_coarse_exposure_value = %d; //ptm_coarse_exposure_value, register configure value", pDataBufferParam->ptm_coarse_exposure_value);
    LOG_DEBUG("UINT8            ptm_fine_exposure_value = %d;   //ptm_fine_exposure_value, register configure value", pDataBufferParam->ptm_fine_exposure_value);
    LOG_DEBUG("UINT8            pcm_gray_exposure_value = %d;   //pcm_gray_exposure_value, register configure value", pDataBufferParam->pcm_gray_exposure_value);
    LOG_DEBUG("UINT8            exposure_period = %d;           //laser_exposure_period, register configure value", pDataBufferParam->exposure_period);
    LOG_DEBUG("UINT64           frame_sequence = %ld;", pDataBufferParam->frame_sequence);
    LOG_DEBUG("UINT64           frame_timestamp_us = %ld;", pDataBufferParam->frame_timestamp_us);
    LOG_DEBUG("UINT16           mipi_rx_fps = %d;", pDataBufferParam->mipi_rx_fps);
    LOG_DEBUG("UINT8            roi_data_index = %d;", pDataBufferParam->roi_data_index);
    LOG_DEBUG("UINT32           buffer_size = %d;               // the size for the following buffer, unit is byte", pDataBufferParam->buffer_size);
    LOG_DEBUG("sizeof(frame_buffer_param_t) = %ld", sizeof(frame_buffer_param_t));
    LOG_DEBUG("------------------------------------------------------");

    return 0;
}

bool Host_Communication::save_data_2_bin_file(const char *prefix, void *buf, int len)
{
    QDateTime       localTime = QDateTime::currentDateTime();
    QString         currentTime = localTime.toString("yyyyMMddhhmmss");
    char *          LocalTimeStr = (char *) currentTime.toStdString().c_str();
    char *          filename = new char[128];

    sprintf(filename, "%s%s_%u_%s.bin", DATA_SAVE_PATH, prefix, len, LocalTimeStr);
    Utils *utils = new Utils();
    utils->save_binary_file(filename, buf, 
        len,
        __FUNCTION__,
        __LINE__
        );
    delete utils;
    delete[] filename;

    return true;
}

int Host_Communication::upload_req_histogram_data(void* pHistDataPtr, uint32_t spotHistDataSize, frame_buffer_param_t *pFrmBufParam)
{
    int result;
    void *async_buf;
    CommandData_t* pCmdData = NULL_POINTER;
    frame_buffer_param_t *pDataBufferParam;
    uint32_t ulCmdDataLen = sizeof(CommandData_t) + sizeof(frame_buffer_param_t);
    uint32_t ulBufSize = ulCmdDataLen + spotHistDataSize;

    if (false == qApp->is_capture_req_from_host())
    {
        return 0;
    }

    if (NULL_POINTER == pHistDataPtr || 0 == spotHistDataSize)
    {
        return 0;
    }

    if (false == connected || false == sender_is_working()) {
        LOG_ERROR("It seems host is not connected yet or disconnected, connected: %d.\n", connected);
        return -1;
    }

    async_buf = sender_async_buf_alloc(ulBufSize);
    if (NULL_POINTER == async_buf) {
        LOG_ERROR("async alloc buf fail: %u.\n", ulBufSize);
        return -1;
    }

    pCmdData = (CommandData_t*) async_buf;

    pCmdData->cmd = CMD_DEVICE_SIDE_UPLOAD_REQ_POS_HISTOGRAM_DATA;
    pDataBufferParam  = (frame_buffer_param_t *) pCmdData->param;
    memcpy(pDataBufferParam, pFrmBufParam, sizeof(frame_buffer_param_t));
    pDataBufferParam->buffer_size = spotHistDataSize;
    memcpy(pDataBufferParam->buffer, pHistDataPtr, spotHistDataSize);

    result = sender_async_send_msg(async_buf, ulBufSize, 0);

    if (result < 0) {
        LOG_ERROR("async send histogram data fail: %d.\n", result);
        return result;
    }

    LOG_DEBUG("%s() sucessfully, mipi_rx_fps = %d fps, req_histogram_x: %u, req_histogram_y: %u, frame_sequence: %ld.\n", 
        __FUNCTION__, pDataBufferParam->mipi_rx_fps, req_histogram_x, req_histogram_y, pDataBufferParam->frame_sequence);

    req_histogram_x = 0; // try to avoid re-send data
    req_histogram_y = 0;

    return 0;
}

int Host_Communication::upload_frame_pointcloud_data(void* pFrameData, uint32_t frameData_size, frame_buffer_param_t *pFrmBufParam)
{
    int dump_times = 0;
    int result;
    void *async_buf;
    CommandData_t* pCmdData = NULL_POINTER;
    frame_buffer_param_t *pDataBufferParam;
    uint32_t ulCmdDataLen = sizeof(CommandData_t) + sizeof(frame_buffer_param_t);
    uint32_t ulBufSize = ulCmdDataLen + frameData_size;
    struct timeval tv;
    long currTimeUsec;
    unsigned long streamed_timeUs;

    if (false == qApp->is_capture_req_from_host())
    {
        return 0;
    }

    if (0 == (get_req_out_data_type() & FRAME_DECODED_POINT_CLOUD))
    {
        return 0;
    }

    if (false == connected || false == sender_is_working()) {
        LOG_ERROR("It seems host is not connected yet or disconnected, connected: %d.\n", connected);
        return -1;
    }

    async_buf = sender_async_buf_alloc(ulBufSize);
    if (NULL_POINTER == async_buf) {
        LOG_ERROR("async alloc buf fail: %u.\n", ulBufSize);
        return -1;
    }

    pCmdData = (CommandData_t*) async_buf;

    pCmdData->cmd = CMD_DEVICE_SIDE_UPLOAD_FRAME_POINTCLOUD_DATA;
    pDataBufferParam  = (frame_buffer_param_t *) pCmdData->param;
    memcpy(pDataBufferParam, pFrmBufParam, sizeof(frame_buffer_param_t));
    pDataBufferParam->buffer_size = frameData_size;
    memcpy(pDataBufferParam->buffer, pFrameData, frameData_size);

    dump_times = Utils::get_env_var_intvalue(ENV_VAR_DUMP_FRAME_PARAM_TIMES);
    if (0 != dump_times && txPointCloudFrameCnt < static_cast<unsigned long>(dump_times))
    {
        dump_frame_param(pDataBufferParam);
    }

    result = sender_async_send_msg(async_buf, ulBufSize, 0);
    gettimeofday(&tv,NULL_POINTER);

    if (result < 0) {
        LOG_ERROR("async send point cloud data fail: %d.\n", result);
        return result;
    }

    txPointCloudFrameCnt++;

    if (0 == firstPointCloudFrameTimeUsec)
    {
        firstPointCloudFrameTimeUsec = tv.tv_sec*1000000 + tv.tv_usec;
    }
    else {
        currTimeUsec = tv.tv_sec*1000000 + tv.tv_usec;
        streamed_timeUs = (currTimeUsec - firstPointCloudFrameTimeUsec);
        txPointCloudFrame_fps = (txPointCloudFrameCnt * 1000000) / streamed_timeUs;
    }

    if (txPointCloudFrameCnt < 3)
    {
        LOG_DEBUG("%s() sucessfully, mipi_rx_fps = %d fps, txPointCloudFrame_fps = %d fps, txPointCloudFrameCnt: %ld, frame_sequence: %ld, sizeof(CommandData_t): %ld, ulCmdDataLen: %d, frameData_size: %d, ulBufSize: %d.\n", 
            __FUNCTION__, pDataBufferParam->mipi_rx_fps, txPointCloudFrame_fps, txPointCloudFrameCnt, pDataBufferParam->frame_sequence, sizeof(CommandData_t), ulCmdDataLen, frameData_size, ulBufSize);
    }

    return 0;
}

int Host_Communication::upload_frame_depth16_data(void* pFrameData, uint32_t frameData_size, frame_buffer_param_t *pFrmBufParam)
{
    int dump_times = 0;
    int result;
    void *async_buf;
    CommandData_t* pCmdData = NULL_POINTER;
    frame_buffer_param_t *pDataBufferParam;
    uint32_t ulCmdDataLen = sizeof(CommandData_t) + sizeof(frame_buffer_param_t);
    uint32_t ulBufSize = ulCmdDataLen + frameData_size;
    struct timeval tv;
    long currTimeUsec;
    unsigned long streamed_timeUs;

    if (false == qApp->is_capture_req_from_host())
    {
        return 0;
    }

    if (0 == (get_req_out_data_type() & FRAME_DECODED_DEPTH16))
    {
        return 0;
    }

    if (false == connected || false == sender_is_working()) {
        LOG_ERROR("It seems host is not connected yet or disconnected, connected: %d.\n", connected);
        return -1;
    }

    async_buf = sender_async_buf_alloc(ulBufSize);
    if (NULL_POINTER == async_buf) {
        LOG_ERROR("async alloc buf fail: %u.\n", ulBufSize);
        return -1;
    }

    pCmdData = (CommandData_t*) async_buf;

    pCmdData->cmd = CMD_DEVICE_SIDE_UPLOAD_FRAME_DEPTH16_DATA;
    pDataBufferParam  = (frame_buffer_param_t *) pCmdData->param;
    memcpy(pDataBufferParam, pFrmBufParam, sizeof(frame_buffer_param_t));
    pDataBufferParam->buffer_size = frameData_size;
    memcpy(pDataBufferParam->buffer, pFrameData, frameData_size);

    dump_times = Utils::get_env_var_intvalue(ENV_VAR_DUMP_FRAME_PARAM_TIMES);
    if (0 != dump_times && txDepth16FrameCnt < static_cast<unsigned long>(dump_times))
    {
        dump_frame_param(pDataBufferParam);
    }

    result = sender_async_send_msg(async_buf, ulBufSize, 0);
    gettimeofday(&tv,NULL_POINTER);

    if (result < 0) {
        LOG_ERROR("async send depth16 data fail: %d.\n", result);
        return result;
    }

    txDepth16FrameCnt++;

    if (0 == firstDepth16FrameTimeUsec)
    {
        firstDepth16FrameTimeUsec = tv.tv_sec*1000000 + tv.tv_usec;
    }
    else {
        currTimeUsec = tv.tv_sec*1000000 + tv.tv_usec;
        streamed_timeUs = (currTimeUsec - firstDepth16FrameTimeUsec);
        txDepth16Frame_fps = (txDepth16FrameCnt * 1000000) / streamed_timeUs;
    }

    //if (1 == txDepth16FrameCnt % FRAME_INTERVAL_4_PROGRESS_REPORT)
    if (txDepth16FrameCnt < 3)
    {
        LOG_DEBUG("%s() sucessfully, mipi_rx_fps = %d fps, txDepth16Frame_fps = %d fps, txDepth16FrameCnt: %ld, frame_sequence: %ld, sizeof(CommandData_t): %ld, ulCmdDataLen: %d, frameData_size: %d, ulBufSize: %d.\n", 
            __FUNCTION__, pDataBufferParam->mipi_rx_fps, txDepth16Frame_fps, txDepth16FrameCnt, pDataBufferParam->frame_sequence, sizeof(CommandData_t), ulCmdDataLen, frameData_size, ulBufSize);
    }

    return 0;
}

int Host_Communication::upload_frame_raw_data(void* pFrameData, uint32_t frameData_size, frame_buffer_param_t *pFrmBufParam)
{
    int dump_times = 0;
    int result;
    void *async_buf;
    CommandData_t* pCmdData = NULL_POINTER;
    frame_buffer_param_t *pDataBufferParam;
    uint32_t ulCmdDataLen = sizeof(CommandData_t) + sizeof(frame_buffer_param_t);
    uint32_t ulBufSize = ulCmdDataLen + frameData_size;
    struct timeval tv;
    long currTimeUsec;
    unsigned long streamed_timeUs;

    if (false == qApp->is_capture_req_from_host())
    {
        return 0;
    }

    if (0 == (get_req_out_data_type() & FRAME_RAW_DATA))
    {
        return 0;
    }

    if (false == connected || false == sender_is_working()) {
        LOG_ERROR("It seems host is not connected yet or disconnected, connected: %d.\n", connected);
        return -1;
    }

    async_buf = sender_async_buf_alloc(ulBufSize);
    if (NULL_POINTER == async_buf) {
        LOG_ERROR("async alloc buf fail: %u.\n", ulBufSize);
        return -1;
    }

    pCmdData = (CommandData_t*) async_buf;

    pCmdData->cmd = CMD_DEVICE_SIDE_UPLOAD_FRAME_RAW_DATA;
    pDataBufferParam  = (frame_buffer_param_t *) pCmdData->param;
    memcpy(pDataBufferParam, pFrmBufParam, sizeof(frame_buffer_param_t));
    pDataBufferParam->buffer_size = frameData_size;
    memcpy(pDataBufferParam->buffer, pFrameData, frameData_size);

    dump_times = Utils::get_env_var_intvalue(ENV_VAR_DUMP_FRAME_PARAM_TIMES);
    if (0 != dump_times && txRawdataFrameCnt < static_cast<unsigned long>(dump_times))
    {
        dump_frame_param(pDataBufferParam);
    }

    result = sender_async_send_msg(async_buf, ulBufSize, 0);
    gettimeofday(&tv,NULL_POINTER);

    if (result < 0) {
        LOG_ERROR("async send raw data fail: %d.\n", result);
        return result;
    }

    txRawdataFrameCnt++;

    if (0 == firstRawdataFrameTimeUsec)
    {
        firstRawdataFrameTimeUsec = tv.tv_sec*1000000 + tv.tv_usec;
    }
    else {
        currTimeUsec = tv.tv_sec*1000000 + tv.tv_usec;
        streamed_timeUs = (currTimeUsec - firstRawdataFrameTimeUsec);
        txRawdataFrame_fps = (txRawdataFrameCnt * 1000000) / streamed_timeUs;
    }

    //if (1 == txRawdataFrameCnt % FRAME_INTERVAL_4_PROGRESS_REPORT)
    if (txRawdataFrameCnt < 3)
    {
        LOG_DEBUG("%s() sucessfully, mipi_rx_fps = %d fps, txRawdataFrame_fps = %d fps, txRawdataFrameCnt: %ld, frame_sequence: %ld, frameData_size: %d, ulBufSize: %d.\n", 
            __FUNCTION__, pDataBufferParam->mipi_rx_fps, txRawdataFrame_fps, txRawdataFrameCnt, pDataBufferParam->frame_sequence, frameData_size, ulBufSize);
    }

    return 0;
}

int Host_Communication::dump_module_static_data(module_static_data_t *pStaticDataParam)
{
    LOG_DEBUG("UINT8            data_type = 0x%x;               // refer to enum swift_data_type of this .h file", pStaticDataParam->data_type);
    LOG_DEBUG("UINT32           module_type = 0x%x;             // refer to ADS6401_MODULE_SPOT and ADS6401_MODULE_SMALL_FLOOD of adaps_types.h file", pStaticDataParam->module_type);
    LOG_DEBUG("UINT32           eeprom_capacity = %d;           // unit is byte", pStaticDataParam->eeprom_capacity);
    LOG_DEBUG("UINT16           otp_vbe25 = 0x%04x;", pStaticDataParam->otp_vbe25);
    LOG_DEBUG("UINT16           otp_vbd = 0x%04x;", pStaticDataParam->otp_vbd);
    LOG_DEBUG("UINT16           otp_adc_vref = 0x%04x;", pStaticDataParam->otp_adc_vref);
    LOG_DEBUG("CHAR             chip_product_id = [%s];", pStaticDataParam->chip_product_id);
    LOG_DEBUG("CHAR             sensor_drv_version = [%s];", pStaticDataParam->sensor_drv_version);
    LOG_DEBUG("CHAR             algo_lib_version = [%s];", pStaticDataParam->algo_lib_version);
    LOG_DEBUG("CHAR             sender_lib_version = [%s];", pStaticDataParam->sender_lib_version);
    LOG_DEBUG("CHAR             spadisQT_version = [%s];", pStaticDataParam->spadisQT_version);
    LOG_DEBUG("UINT32           eeprom_data_size = %d;          // unit is byte", pStaticDataParam->eeprom_data_size);
    LOG_DEBUG("sizeof(module_static_data_t) = %ld", sizeof(module_static_data_t));

    return 0;
}

int Host_Communication::upload_module_static_data()
{
    uint32_t ulBufSize;
    void *async_buf;
    void* pEEPROM_buffer;
    uint32_t eeprom_data_size;
    void *module_static_data_addr;
    struct adaps_dtof_module_static_data *module_static_data;
    module_static_data_t *pStaticDataParam;
    CommandData_t* pCmdData;
    uint32_t ulCmdDataLen = sizeof(CommandData_t) + sizeof(module_static_data_t);

    if (false == connected || false == sender_is_working()) {
        LOG_ERROR("It seems host is not connected yet or disconnected, connected: %d.\n", connected);
        return -1;
    }

    p_misc_device = qApp->get_misc_dev_instance();

    if (NULL_POINTER == p_misc_device)
    {
        DBG_ERROR("p_misc_device is NULL, p_misc_device:%p, qApp: %p, qApp->get_misc_dev_instance(): %p", p_misc_device, qApp, qApp->get_misc_dev_instance());
        return -1;
    }

    qApp->set_capture_req_from_host(true);
    p_misc_device->get_dtof_module_static_data(&module_static_data_addr, &pEEPROM_buffer, &eeprom_data_size);
    module_static_data = (struct adaps_dtof_module_static_data *) module_static_data_addr;
    ulBufSize = ulCmdDataLen + eeprom_data_size;

    async_buf = sender_async_buf_alloc(ulBufSize);
    if (NULL_POINTER == async_buf) {
        LOG_ERROR("async alloc buf fail: %u.\n", ulBufSize);
        return -1;
    }

    pCmdData = (CommandData_t*) async_buf;

    pCmdData->cmd = CMD_DEVICE_SIDE_UPLOAD_MODULE_STATIC_DATA;
    pStaticDataParam  = (module_static_data_t *) pCmdData->param;
    pStaticDataParam->module_type = module_static_data->module_type;
    pStaticDataParam->eeprom_capacity = module_static_data->eeprom_capacity;
    eeprom_capacity = module_static_data->eeprom_capacity;
    pStaticDataParam->otp_vbe25 = module_static_data->otp_vbe25;
    pStaticDataParam->otp_vbd = module_static_data->otp_vbd;
    pStaticDataParam->otp_adc_vref = module_static_data->otp_adc_vref;
    memcpy(pStaticDataParam->chip_product_id, module_static_data->chip_product_id, SWIFT_PRODUCT_ID_SIZE);

    memcpy(pStaticDataParam->sensor_drv_version, module_static_data->sensor_drv_version, FW_VERSION_LENGTH);
    DepthMapWrapperGetVersion(pStaticDataParam->algo_lib_version);
    strcpy(pStaticDataParam->sender_lib_version, sender_get_version_str());
    sprintf(pStaticDataParam->spadisQT_version, "%d.%d.%d", VERSION_MAJOR, VERSION_MINOR, VERSION_REVISION);

    pStaticDataParam->data_type = MODULE_STATIC_DATA;
    pStaticDataParam->eeprom_data_size = eeprom_data_size;
    memcpy(pStaticDataParam->eeprom_data, pEEPROM_buffer, eeprom_data_size);
    dump_buffer_data(pStaticDataParam->eeprom_data, "eeprom_data_to_PC", __LINE__);
    dump_module_static_data(pStaticDataParam);

    int result = sender_async_send_msg(async_buf, ulBufSize, 0);

    if (result < 0) {
        //sender_async_buf_free(pstrCmdData);
        LOG_ERROR("async send module calibrate data fail: %u.\n", ulBufSize);
        return result;
    }
    LOG_DEBUG("%s() sucessfully, send_buffer_size: %d.\n", __FUNCTION__, ulBufSize);

    if (false == module_static_data->eeprom_crc_matched) {
        char err_msg[] = "There is a/some mismatched CRC checksum for eeprom data, please consider to correct it.";
        report_status(CMD_HOST_SIDE_GET_MODULE_STATIC_DATA, CMD_DEVICE_SIDE_ERROR_CHKSUM_MISMATCH_IN_EEPROM, err_msg, strlen(err_msg));
    }
    return 0;
}

int Host_Communication::dump_capture_req_param(capture_req_param_t* pCaptureReqParam)
{
    LOG_DEBUG("UINT8                    work_mode = %d;             // refer to swift_workmode_t of adaps_types.h", pCaptureReqParam->work_mode);
    LOG_DEBUG("UINT8                    env_type = %d;              // refer to AdapsEnvironmentType of adaps_types.h", pCaptureReqParam->env_type);
    LOG_DEBUG("UINT8                    measure_type = %d;          // refer to AdapsMeasurementType of adaps_types.h", pCaptureReqParam->measure_type);
    LOG_DEBUG("UINT8                    framerate_type = %d;        // refer to AdapsFramerateType of adaps_types.h", pCaptureReqParam->framerate_type);
    LOG_DEBUG("UINT8                    power_mode = %d;            // refer to AdapsPowerMode of adaps_types.h", pCaptureReqParam->power_mode);
    LOG_DEBUG("UINT8                    walkerror_enable = %d;", pCaptureReqParam->walkerror_enable);
    LOG_DEBUG("UINT8                    req_out_data_type = %d;     // refer to enum swift_data_type of this .h file", pCaptureReqParam->req_out_data_type);
    LOG_DEBUG("BOOLEAN                  compose_subframe = %d;", pCaptureReqParam->compose_subframe);
    LOG_DEBUG("BOOLEAN                  expand_pixel = %d;", pCaptureReqParam->expand_pixel);
    LOG_DEBUG("BOOLEAN                  mirror_x = %d;", pCaptureReqParam->mirror_x);
    LOG_DEBUG("BOOLEAN                  mirror_y = %d;", pCaptureReqParam->mirror_y);
    LOG_DEBUG("BOOLEAN                  laserEnable = %d;", pCaptureReqParam->laserEnable);
    LOG_DEBUG("BOOLEAN                  vopAdjustEnable = %d;", pCaptureReqParam->vopAdjustEnable);
    LOG_DEBUG("UINT8                    rowSearchingRange = %d;", pCaptureReqParam->rowSearchingRange);
    LOG_DEBUG("UINT8                    colSearchingRange = %d;", pCaptureReqParam->colSearchingRange);
    LOG_DEBUG("UINT8                    rowOffset = %d;", pCaptureReqParam->rowOffset);
    LOG_DEBUG("UINT8                    colOffset = %d;", pCaptureReqParam->colOffset);
    LOG_DEBUG("exposure_time_param_t    exposure_param = (0x%02x, 0x%02x, 0x%02x);", 
            pCaptureReqParam->expose_param.coarseExposure, pCaptureReqParam->expose_param.fineExposure, pCaptureReqParam->expose_param.grayExposure);
    LOG_DEBUG("BOOLEAN                  roi_sram_rolling = %d;", pCaptureReqParam->roi_sram_rolling);
    LOG_DEBUG("BOOLEAN                  script_loaded = %d;", pCaptureReqParam->script_loaded);
    LOG_DEBUG("UINT32                   script_size = %d;           // set to 0 if script_loaded == false", pCaptureReqParam->script_size);
    LOG_DEBUG("sizeof(capture_req_param_t) = %ld", sizeof(capture_req_param_t));

    return 0;
}

void Host_Communication::adaps_set_colormap_range(CommandData_t* pCmdData, uint32_t rxDataLen)
{
    colormap_range_param_t* pColormapRangeParam;

    if (rxDataLen < (sizeof(CommandData_t) + sizeof(colormap_range_param_t)))
    {
        LOG_ERROR("swift_set_colormap_range: rxDataLen %d is too short for CMD_HOST_SIDE_SET_COLORMAP_RANGE_PARAM.\n", rxDataLen);
        return;
    }

    pColormapRangeParam = (colormap_range_param_t*) pCmdData->param;

    qApp->set_GrayScaleMinMappedRange(pColormapRangeParam->GrayScaleMinMappedRange);
    qApp->set_GrayScaleMaxMappedRange(pColormapRangeParam->GrayScaleMaxMappedRange);

    qApp->set_RealDistanceMinMappedRange(pColormapRangeParam->RealDistanceMinMappedRange);
    qApp->set_RealDistanceMaxMappedRange(pColormapRangeParam->RealDistanceMaxMappedRange);
}

void Host_Communication::adaps_set_walkerror_enable(CommandData_t* pCmdData, uint32_t rxDataLen)
{
    walkerror_enable_param_t* pWalkerrorEnableParam;

    if (rxDataLen < (sizeof(CommandData_t) + sizeof(walkerror_enable_param_t)))
    {
        LOG_ERROR("swift_set_walkerror_enable: rxDataLen %d is too short for CMD_HOST_SIDE_SET_WALKERROR_ENABLE.\n", rxDataLen);
        return;
    }

    pWalkerrorEnableParam = (walkerror_enable_param_t*) pCmdData->param;

    qApp->set_walkerror_enable(pWalkerrorEnableParam->walkerror_enable);
}

void Host_Communication::adaps_set_req_histogram_position(CommandData_t* pCmdData, uint32_t rxDataLen)
{
    histogram_report_param_t* pReqHistPosParam;

    if (rxDataLen < (sizeof(CommandData_t) + sizeof(histogram_report_param_t)))
    {
        LOG_ERROR("adaps_set_req_histogram_position: rxDataLen %d is too short for CMD_HOST_SIDE_SET_HISTOGRAM_DATA_REQ_POSITION.\n", rxDataLen);
        return;
    }

    pReqHistPosParam = (histogram_report_param_t*) pCmdData->param;

    req_histogram_x = pReqHistPosParam->x;
    req_histogram_y = pReqHistPosParam->y;
}

void Host_Communication::download_roisram_data(CommandData_t* pCmdData, uint32_t rxDataLen)
{
    roisram_data_param_t* pRoiSramParam;
    char msg[] = "roi sram loaded successfully.";

    if (rxDataLen < (sizeof(CommandData_t) + sizeof(roisram_data_param_t)))
    {
        LOG_ERROR("<%s>: rxDataLen %d is too short for CMD_HOST_SIDE_DOWNLOAD_ROI_SRAM_DATA.\n", __FUNCTION__, rxDataLen);
        return;
    }

    pRoiSramParam = (roisram_data_param_t*) pCmdData->param;
    if (pRoiSramParam->roisram_data_size > 0 && 0 == (pRoiSramParam->roisram_data_size % PER_ROISRAM_GROUP_SIZE))
    {
        UINT8 *mmaped_roisram_address = qApp->get_mmap_address_4_loaded_roisram();
        qApp->set_size_4_loaded_roisram(pRoiSramParam->roisram_data_size);
        qApp->set_capture_req_from_host(true);

        LOG_DEBUG("------loaded_roisram_data %d bytes-----\n", pRoiSramParam->roisram_data_size);
        memcpy(mmaped_roisram_address, &pRoiSramParam->roisram_data, pRoiSramParam->roisram_data_size);

        if (Utils::is_env_var_true(ENV_VAR_SAVE_LOADED_DATA_ENABLE))
        {
            save_data_2_bin_file("loaded_roisram", mmaped_roisram_address, pRoiSramParam->roisram_data_size);
        }
    }
    else {
        char err_msg[128];
        sprintf(err_msg, "Invalid roisram_data_size(%d) for CMD_HOST_SIDE_DOWNLOAD_ROI_SRAM_DATA", pRoiSramParam->roisram_data_size);
        report_status(CMD_HOST_SIDE_DOWNLOAD_ROI_SRAM_DATA, CMD_DEVICE_SIDE_ERROR_INVALID_ROI_SRAM_SIZE, err_msg, strlen(err_msg));
        return;
    }

    report_status(CMD_HOST_SIDE_DOWNLOAD_ROI_SRAM_DATA, CMD_DEVICE_SIDE_NO_ERROR, msg, strlen(msg));
}

void Host_Communication::download_walkerror_data(CommandData_t* pCmdData, uint32_t rxDataLen)
{
    spot_walkerror_data_param_t* pWalkerrorParam;
    char msg[] = "Walkerror data loaded successfully.";

    if (rxDataLen < (sizeof(CommandData_t) + sizeof(spot_walkerror_data_param_t)))
    {
        LOG_ERROR("<%s>: rxDataLen %d is too short for CMD_HOST_SIDE_DOWNLOAD_SPOT_WALKERROR_DATA.\n", __FUNCTION__, rxDataLen);
        return;
    }

    pWalkerrorParam = (spot_walkerror_data_param_t*) pCmdData->param;
    if (pWalkerrorParam->walkerror_data_size > 0)
    {
        qApp->set_capture_req_from_host(true);

        loaded_walkerror_data_size = pWalkerrorParam->walkerror_data_size;
        
        if (NULL_POINTER == loaded_walkerror_data)
        {
            loaded_walkerror_data = (u8 *) malloc(loaded_walkerror_data_size);
            if (NULL_POINTER == loaded_walkerror_data) {
                DBG_ERROR("Fail to malloc for loaded_walkerror_data.\n");
                return ;
            }
        }
        
        memcpy(loaded_walkerror_data, &pWalkerrorParam->walkerror_data, loaded_walkerror_data_size);
        qApp->set_loaded_walkerror_data(loaded_walkerror_data);
        qApp->set_loaded_walkerror_data_size(loaded_walkerror_data_size);
        LOG_DEBUG("------loaded_walkerror_data %d bytes-----\n", loaded_walkerror_data_size);

        if (Utils::is_env_var_true(ENV_VAR_SAVE_LOADED_DATA_ENABLE))
        {
            save_data_2_bin_file("loaded_walkerror_data", loaded_walkerror_data, loaded_walkerror_data_size);
        }
    }
    else {
        char err_msg[128];
        sprintf(err_msg, "Invalid walkerror_data_size(%d) for CMD_HOST_SIDE_DOWNLOAD_SPOT_WALKERROR_DATA", pWalkerrorParam->walkerror_data_size);
        report_status(CMD_HOST_SIDE_DOWNLOAD_SPOT_WALKERROR_DATA, CMD_DEVICE_SIDE_ERROR_INVALID_WALKERROR_SIZE, err_msg, strlen(err_msg));
        return;
    }


    report_status(CMD_HOST_SIDE_DOWNLOAD_SPOT_WALKERROR_DATA, CMD_DEVICE_SIDE_NO_ERROR, msg, strlen(msg));
}

void Host_Communication::download_spotoffset_data(CommandData_t* pCmdData, uint32_t rxDataLen)
{
    spot_offset_data_param_t* pWalkerrorParam;
    char msg[] = "Offset data loaded successfully.";

    if (rxDataLen < (sizeof(CommandData_t) + sizeof(spot_offset_data_param_t)))
    {
        LOG_ERROR("<%s>: rxDataLen %d is too short for CMD_HOST_SIDE_DOWNLOAD_SPOT_OFFSET_DATA.\n", __FUNCTION__, rxDataLen);
        return;
    }

    pWalkerrorParam = (spot_offset_data_param_t*) pCmdData->param;
    if (pWalkerrorParam->offset_data_size > 0)
    {
        qApp->set_capture_req_from_host(true);

        loaded_spotoffset_data_size = pWalkerrorParam->offset_data_size;
        
        if (NULL_POINTER == loaded_spotoffset_data)
        {
            loaded_spotoffset_data = (u8 *) malloc(loaded_spotoffset_data_size);
            if (NULL_POINTER == loaded_spotoffset_data) {
                DBG_ERROR("Fail to malloc for loaded_spotoffset_data.\n");
                return ;
            }
        }
        
        memcpy(loaded_spotoffset_data, &pWalkerrorParam->offset_data, loaded_spotoffset_data_size);
        qApp->set_loaded_spotoffset_data(loaded_spotoffset_data);
        qApp->set_loaded_spotoffset_data_size(loaded_spotoffset_data_size);
        LOG_DEBUG("------loaded_spotoffset_data %d bytes-----\n", loaded_spotoffset_data_size);

        if (Utils::is_env_var_true(ENV_VAR_SAVE_LOADED_DATA_ENABLE))
        {
            save_data_2_bin_file("loaded_offset_data", loaded_spotoffset_data, loaded_spotoffset_data_size);
        }
    }
    else {
        char err_msg[128];
        sprintf(err_msg, "Invalid spotoffset_data_size(%d) for CMD_HOST_SIDE_DOWNLOAD_SPOT_OFFSET_DATA", pWalkerrorParam->offset_data_size);
        report_status(CMD_HOST_SIDE_DOWNLOAD_SPOT_OFFSET_DATA, CMD_DEVICE_SIDE_ERROR_INVALID_SPOTOFFSET_SIZE, err_msg, strlen(err_msg));
        return;
    }


    report_status(CMD_HOST_SIDE_DOWNLOAD_SPOT_OFFSET_DATA, CMD_DEVICE_SIDE_NO_ERROR, msg, strlen(msg));
}

void Host_Communication::download_ref_distance_data(CommandData_t* pCmdData, uint32_t rxDataLen)
{
    UINT32 loaded_ref_distance_data_size = sizeof(reference_distance_data_param_t);
    reference_distance_data_param_t* pRefDistanceParam;
    char msg[] = "Ref distance data loaded successfully.";

    if (rxDataLen < (sizeof(CommandData_t) + sizeof(reference_distance_data_param_t)))
    {
        LOG_ERROR("<%s>: rxDataLen %d is too short for CMD_HOST_SIDE_DOWNLOAD_REF_DISTANCE_DATA.\n", __FUNCTION__, rxDataLen);
        return;
    }

    pRefDistanceParam = (reference_distance_data_param_t*) pCmdData->param;

    qApp->set_capture_req_from_host(true);

    if (NULL_POINTER == loaded_ref_distance_data)
    {
        loaded_ref_distance_data = (float *) malloc(loaded_ref_distance_data_size);
        if (NULL_POINTER == loaded_ref_distance_data) {
            DBG_ERROR("Fail to malloc for loaded_ref_distance_data.\n");
            return ;
        }
    }
    
    memcpy(loaded_ref_distance_data, pRefDistanceParam, loaded_ref_distance_data_size);
    //qApp->set_loaded_ref_distance_data(loaded_ref_distance_data);
    //qApp->set_loaded_ref_distance_data_size(loaded_ref_distance_data_size);
    LOG_DEBUG("------loaded_ref_distance_data %d bytes-----\n", loaded_ref_distance_data_size);

    if (Utils::is_env_var_true(ENV_VAR_SAVE_LOADED_DATA_ENABLE))
    {
        save_data_2_bin_file("loaded_refdistance_data", loaded_ref_distance_data, loaded_ref_distance_data_size);
    }

    report_status(CMD_HOST_SIDE_DOWNLOAD_REF_DISTANCE_DATA, CMD_DEVICE_SIDE_NO_ERROR, msg, strlen(msg));
}

void Host_Communication::download_lens_intrinsic_data(CommandData_t* pCmdData, uint32_t rxDataLen)
{
    UINT32 loaded_lens_intrinsic_data_size = sizeof(lens_intrinsic_data_param_t);
    lens_intrinsic_data_param_t* pLensIntrinsicParam;
    char msg[] = "lens intrinsic data loaded successfully.";

    if (rxDataLen < (sizeof(CommandData_t) + sizeof(lens_intrinsic_data_param_t)))
    {
        LOG_ERROR("<%s>: rxDataLen %d is too short for CMD_HOST_SIDE_DOWNLOAD_LENS_INTRINSIC_DATA.\n", __FUNCTION__, rxDataLen);
        return;
    }

    pLensIntrinsicParam = (lens_intrinsic_data_param_t*) pCmdData->param;

    qApp->set_capture_req_from_host(true);

    if (NULL_POINTER == loaded_lens_intrinsic_data)
    {
        loaded_lens_intrinsic_data = (float *) malloc(loaded_lens_intrinsic_data_size);
        if (NULL_POINTER == loaded_lens_intrinsic_data) {
            DBG_ERROR("Fail to malloc for loaded_lens_intrinsic_data.\n");
            return ;
        }
    }
    
    memcpy(loaded_lens_intrinsic_data, pLensIntrinsicParam, loaded_lens_intrinsic_data_size);
    //qApp->set_loaded_ref_distance_data(loaded_ref_distance_data);
    //qApp->set_loaded_ref_distance_data_size(loaded_ref_distance_data_size);
    LOG_DEBUG("------loaded_lens_intrinsic_data %d bytes-----\n", loaded_lens_intrinsic_data_size);

    if (Utils::is_env_var_true(ENV_VAR_SAVE_LOADED_DATA_ENABLE))
    {
        save_data_2_bin_file("loaded_lens_intrinsic_data", loaded_lens_intrinsic_data, loaded_lens_intrinsic_data_size);
    }

    report_status(CMD_HOST_SIDE_DOWNLOAD_LENS_INTRINSIC_DATA, CMD_DEVICE_SIDE_NO_ERROR, msg, strlen(msg));
}

void Host_Communication::adaps_request_device_reboot(CommandData_t* pCmdData, uint32_t rxDataLen)
{
    device_reboot_request_t* pRebootParam;

    if (rxDataLen < (sizeof(CommandData_t) + sizeof(device_reboot_request_t)))
    {
        LOG_ERROR("<%s>: rxDataLen %d is too short for CMD_HOST_SIDE_SET_DEVICE_REBOOT.\n", __FUNCTION__, rxDataLen);
        return;
    }

    pRebootParam = (device_reboot_request_t*) pCmdData->param;
    if (CMD_HOST_SIDE_REBOOT_NO_REASON != pRebootParam->reboot_reason_code)
    {
        LOG_DEBUG("System will re-boot now.\n");
        LOG_DEBUG("Request reboot reason: %s\n", pRebootParam->reboot_reason_msg);
        Utils::system_reboot();
    }
    else {
        char err_msg[128];
        sprintf(err_msg, "Invalid reboot_reason_code(%d) for CMD_HOST_SIDE_SET_DEVICE_REBOOT", pRebootParam->reboot_reason_code);
        report_status(CMD_HOST_SIDE_SET_DEVICE_REBOOT, CMD_DEVICE_SIDE_ERROR_INVALID_REBOOT_REASON, err_msg, strlen(err_msg));
        return;
    }
}

void Host_Communication::adaps_set_module_kernel_type(CommandData_t* pCmdData, uint32_t rxDataLen)
{
    module_kernel_param_t* pKernelParam;

    if (rxDataLen < (sizeof(CommandData_t) + sizeof(module_kernel_param_t)))
    {
        LOG_ERROR("<%s>: rxDataLen %d is too short for CMD_HOST_SIDE_SET_MODULE_KERNEL_TYPE.\n", __FUNCTION__, rxDataLen);
        return;
    }

    pKernelParam = (module_kernel_param_t*) pCmdData->param;
    if (pKernelParam->module_kernel_type < MODULE_KERNEL_TYPE_CNT)
    {
        qApp->set_module_kernel_type(pKernelParam->module_kernel_type);
    }
    else {
        char err_msg[128];
        sprintf(err_msg, "Invalid module_kernel_type(%d) for CMD_HOST_SIDE_SET_MODULE_KERNEL_TYPE", pKernelParam->module_kernel_type);
        report_status(CMD_HOST_SIDE_SET_MODULE_KERNEL_TYPE, CMD_DEVICE_SIDE_ERROR_INVALID_MODULE_KERNEL_TYPE, err_msg, strlen(err_msg));
        return;
    }
}

void Host_Communication::adaps_update_eeprom_data(CommandData_t* pCmdData, uint32_t rxDataLen)
{
    int ret = -1;
    UINT32 update_data_size;
    UINT32 update_data_offset;
    UINT8 *update_data;
    eeprom_data_update_param_t* pEepromUpdateParam;

    if (rxDataLen < (sizeof(CommandData_t) + sizeof(eeprom_data_update_param_t)))
    {
        LOG_ERROR("<%s>: rxDataLen %d is too short for CMD_HOST_SIDE_UPDATE_EEPROM_DATA.\n", __FUNCTION__, rxDataLen);
        return;
    }

    qApp->set_capture_req_from_host(true);
    pEepromUpdateParam = (eeprom_data_update_param_t*) pCmdData->param;
    update_data_size = pEepromUpdateParam->length;
    update_data_offset = pEepromUpdateParam->offset;
    update_data = (UINT8 *) pEepromUpdateParam->eeprom_data;

    if ((update_data_size > 0) && ((0 != eeprom_capacity) && (eeprom_capacity >= (update_data_offset + update_data_size))))
    {
        p_misc_device = qApp->get_misc_dev_instance();
        dump_buffer_data(update_data, "update_eeprom_data", __LINE__);
        ret = p_misc_device->update_eeprom_data(update_data, update_data_offset, update_data_size);
        LOG_DEBUG("------update_eeprom_data_size %d, offset: 0x%x/%d, eeprom_capacity: %d-----\n", update_data_size, update_data_offset, update_data_offset, eeprom_capacity);
    }
    else {
        char err_msg[128];
        sprintf(err_msg, "Invalid parameter(offset: %d, length: %d) for CMD_HOST_SIDE_UPDATE_EEPROM_DATA", update_data_offset, update_data_size);
        report_status(CMD_HOST_SIDE_UPDATE_EEPROM_DATA, CMD_DEVICE_SIDE_ERROR_INVALID_EEPROM_UPD_PARAM, err_msg, strlen(err_msg));
        return;
    }

    if (0 == ret)
    {
        char msg[128];
        sprintf(msg, "update %d bytes eeprom data successfully.", update_data_size);
        report_status(CMD_HOST_SIDE_UPDATE_EEPROM_DATA, CMD_DEVICE_SIDE_NO_ERROR, msg, strlen(msg));
    }
    else {
        char msg[128];
        sprintf(msg, "Fail to update %d bytes eeprom data, check kernel log please.", update_data_size);
        report_status(CMD_HOST_SIDE_UPDATE_EEPROM_DATA, CMD_DEVICE_SIDE_ERROR_FAIL_TO_UPDATE_EEPROM, msg, strlen(msg));
    }

}

void Host_Communication::adaps_set_rtc_time(CommandData_t* pCmdData, uint32_t rxDataLen)
{
    rtc_time_param_t* pRtcTimeParam;
    rtc_time_param_t strTimeSyncInfo;
    struct timeval strLocalTime;

    if (rxDataLen < (sizeof(CommandData_t) + sizeof(rtc_time_param_t)))
    {
        LOG_ERROR("adaps_set_rtc_time: rxDataLen %d is too short for CMD_HOST_SIDE_SET_RTC_TIME.\n", rxDataLen);
        return;
    }

    pRtcTimeParam = (rtc_time_param_t*) pCmdData->param;

    gettimeofday(&strLocalTime, NULL);

    //LOG_DEBUG("local time seconds since 00:00:00, 1 Jan 1970 UTC: %ld.\n", strLocalTime.tv_sec);

    memcpy(&strTimeSyncInfo, pRtcTimeParam, sizeof(rtc_time_param_t));

    //LOG_DEBUG("update time seconds: %ld timezone: %d.\n", strTimeSyncInfo.strUtcTime.tv_sec, strTimeSyncInfo.strTimeZone.tz_minuteswest);
    strTimeSyncInfo.strUtcTime.tv_sec += strTimeSyncInfo.strTimeZone.tz_minuteswest * 60;

    // update system time
    if (strTimeSyncInfo.strUtcTime.tv_sec != strLocalTime.tv_sec) {
        if (settimeofday(&strTimeSyncInfo.strUtcTime, NULL) != 0) {
            LOG_ERROR("call settimeofday fail: %d, %s.\n", errno, strerror(errno));
            return;
        }
    }

}

void Host_Communication::adaps_start_capture(CommandData_t* pCmdData, uint32_t rxDataLen)
{
    capture_req_param_t* pCaptureReqParam;
    UINT8 force_row_search_range = 0;
    UINT8 force_column_search_range = 0;
    UINT8 force_coarseExposure = 0;
    UINT8 force_fineExposure = 0;
    UINT8 force_grayExposure = 0;
    UINT8 force_laserExposurePeriod = 0;
    UINT8 force_framerate_fps = 0;

    if (rxDataLen < (sizeof(CommandData_t) + sizeof(capture_req_param_t)))
    {
        LOG_ERROR("swift_start_capture: rxDataLen %d is too short for CMD_HOST_SIDE_START_CAPTURE.\n", rxDataLen);
        return;
    }

    pCaptureReqParam = (capture_req_param_t*) pCmdData->param;
    LOG_DEBUG("---req_out_data_type: 0x%x, anchorX = %d, anchorY = %d, roi_sram_rolling = %d---\n",
        pCaptureReqParam->req_out_data_type, pCaptureReqParam->colOffset, pCaptureReqParam->rowOffset, pCaptureReqParam->roi_sram_rolling);

    force_row_search_range = Utils::get_env_var_intvalue(ENV_VAR_FORCE_ROW_SEARCH_RANGE);
    if (force_row_search_range)
    {
        pCaptureReqParam->rowSearchingRange = force_row_search_range;
    }

    force_column_search_range = Utils::get_env_var_intvalue(ENV_VAR_FORCE_COLUMN_SEARCH_RANGE);
    if (force_column_search_range)
    {
        pCaptureReqParam->colSearchingRange = force_column_search_range;
    }

    force_coarseExposure = Utils::get_env_var_intvalue(ENV_VAR_FORCE_COARSE_EXPOSURE);
    if (force_coarseExposure)
    {
        pCaptureReqParam->expose_param.coarseExposure = force_coarseExposure;
    }

    force_fineExposure = Utils::get_env_var_intvalue(ENV_VAR_FORCE_FINE_EXPOSURE);
    if (force_fineExposure)
    {
        pCaptureReqParam->expose_param.fineExposure = force_fineExposure;
    }

    force_grayExposure = Utils::get_env_var_intvalue(ENV_VAR_FORCE_GRAY_EXPOSURE);
    if (force_grayExposure)
    {
        pCaptureReqParam->expose_param.grayExposure = force_grayExposure;
    }

    force_framerate_fps = Utils::get_env_var_intvalue(ENV_VAR_FORCE_FRAMERATE_FPS);
    if (force_framerate_fps)
    {
        switch (force_framerate_fps)
        {
            case 15:
                pCaptureReqParam->framerate_type = AdapsFramerateType15FPS;
                break;

            case 25:
                pCaptureReqParam->framerate_type = AdapsFramerateType25FPS;
                break;

            case 60:
                pCaptureReqParam->framerate_type = AdapsFramerateType60FPS;
                break;

            default:
                pCaptureReqParam->framerate_type = AdapsFramerateType30FPS;
                break;
        }
    }

    force_laserExposurePeriod = Utils::get_env_var_intvalue(ENV_VAR_FORCE_LASEREXPOSUREPERIOD);

    if (true == Utils::is_env_var_true(ENV_VAR_DUMP_CAPTURE_REQ_PARAM))
    {
        dump_capture_req_param(pCaptureReqParam);
    }

    qApp->set_capture_req_from_host(true);
    if (ADS6401_MODULE_SPOT == qApp->get_module_type())
    {
        // ONLY spot module need anchor preprocess, according to XiaLing's comment
        qApp->set_anchorOffset(pCaptureReqParam->rowOffset, pCaptureReqParam->colOffset);
    }
    else {
        qApp->set_anchorOffset(0, 0); // non-spot module does not need anchor preprocess
    }
    qApp->set_spotSearchingRange(pCaptureReqParam->rowSearchingRange, pCaptureReqParam->colSearchingRange);
    qApp->set_usrCfgExposureValues(pCaptureReqParam->expose_param.coarseExposure, pCaptureReqParam->expose_param.fineExposure, pCaptureReqParam->expose_param.grayExposure, force_laserExposurePeriod);
    qApp->set_roi_sram_rolling(pCaptureReqParam->roi_sram_rolling);

    memcpy(&backuped_capture_req_param, pCaptureReqParam, sizeof(capture_req_param_t));
    emit set_capture_options(pCaptureReqParam);
    //LOG_ERROR("param.env_type %d.\n", pCaptureReqParam->env_type);
    //pCaptureReqParam->env_type = AdapsEnvTypeIndoor;
    backuped_wkmode = pCaptureReqParam->work_mode;

    if (pCaptureReqParam->script_loaded && pCaptureReqParam->script_size)
    {
        backuped_script_buffer_size = pCaptureReqParam->script_size;

        if (NULL_POINTER == backuped_script_buffer)
        {
            backuped_script_buffer = (u8 *) malloc(backuped_script_buffer_size);
            LOG_DEBUG("------backuped_script_buffer_size: %d, backuped_script_buffer: %p-----\n", backuped_script_buffer_size, backuped_script_buffer);
            if (NULL_POINTER == backuped_script_buffer) {
                DBG_ERROR("Fail to malloc for backup_script_buffer.\n");
                return ;
            }
        }

        memcpy(backuped_script_buffer, pCaptureReqParam->script_buffer, backuped_script_buffer_size);
    }
    else {
        backuped_script_buffer_size = 0;
    }

    emit start_capture();
}

void Host_Communication::get_backuped_external_config_script(UINT8 *workmode, UINT8 ** script_buffer, uint32_t *script_buffer_size)
{
    *workmode = backuped_wkmode;
    *script_buffer_size = backuped_script_buffer_size;
    *script_buffer = backuped_script_buffer;
}

void Host_Communication::read_device_register(UINT16 cmd, CommandData_t* pCmdData, uint32_t rxDataLen)
{
    int ret = 0;
    register_op_data_t *register_op_data;

    LOG_INFO("--- receive report sensor register event -----\n");

    if (rxDataLen != (sizeof(CommandData_t) + sizeof(register_op_data_t)))
    {
        LOG_ERROR("dataLen(%u) is illegal, sizeof(register_op_data_t): %ld.\n", rxDataLen, sizeof(register_op_data_t));
        return;
    }

    register_op_data = (register_op_data_t*) pCmdData->param;
    p_misc_device = qApp->get_misc_dev_instance();
    if (NULL_POINTER == p_misc_device)
    {
        DBG_ERROR("p_misc_device is NULL");
        return;
    }

    ret = p_misc_device->read_device_register(register_op_data);
    if (0 > ret)
    {
        char err_msg[] = "Fail to read register, please check the device side's log for more details.";
        report_status(cmd, CMD_DEVICE_SIDE_ERROR_READ_REGISTER, err_msg, strlen(err_msg));
        return;
    }

    if (CMD_HOST_SIDE_GET_VCSLDRV_OP7020_REGISTER == cmd)
    {
        pCmdData->cmd       = CMD_DEVICE_SIDE_REPORT_VCSLDRV_OP7020_REGISTER;
    }
    else {
        pCmdData->cmd       = CMD_DEVICE_SIDE_REPORT_SENSOR_REGISTER;
    }

    LOG_WARN("get sensor register[0x%x] = 0x%x for i2c_address: 0x%x.\n"
        , register_op_data->reg_addr, register_op_data->reg_val, register_op_data->i2c_address);

    ret = sender_send_msg((void *)pCmdData, (sizeof(CommandData_t) + sizeof(register_op_data_t)));
    if (ret) {
        LOG_ERROR("call sender_send_msg: %u fail: %u.\n", pCmdData->cmd, ret);
    }

    return;
}

void Host_Communication::write_device_register(UINT16 cmd, CommandData_t* pCmdData, uint32_t rxDataLen)
{
    int ret = 0;
    register_op_data_t *register_op_data;

    if (rxDataLen != (sizeof(CommandData_t) + sizeof(register_op_data_t)))
    {
        LOG_ERROR("dataLen(%u) is illegal.\n", rxDataLen);
        return;
    }

    register_op_data = (register_op_data_t*) pCmdData->param;
    LOG_WARN("--- receive write sensor register event, register[0x%x] = 0x%x for i2c_address: 0x%x -----\n"
        , register_op_data->reg_addr, register_op_data->reg_val, register_op_data->i2c_address);

    p_misc_device = qApp->get_misc_dev_instance();
    if (NULL_POINTER == p_misc_device)
    {
        DBG_ERROR("p_misc_device is NULL_POINTER");
        return;
    }

    ret = p_misc_device->write_device_register(register_op_data);
    if (0 > ret)
    {
        char err_msg[] = "Fail to write register, please check the device side's log for more details.";
        report_status(cmd, CMD_DEVICE_SIDE_ERROR_WRITE_REGISTER, err_msg, strlen(err_msg));
    }

    return;
}

void Host_Communication::adaps_event_process(void* pRXData, uint32_t rxDataLen)
{
    V4L2 *v4l2 = qApp->get_v4l2_instance();

    if (rxDataLen < sizeof(CommandData_t)) {
        LOG_ERROR("swift_event_process: rxDataLen %d is too short.\n", rxDataLen);
        return;
    }

    CommandData_t* pCmdData = (CommandData_t*)pRXData;
    LOG_DEBUG("adaps_event_process: cmd = 0x%x, rxDataLen %d.\n", pCmdData->cmd, rxDataLen);
    switch (pCmdData->cmd)
    {
        case CMD_HOST_SIDE_SET_RTC_TIME:
            adaps_set_rtc_time(pCmdData, rxDataLen);
            break;

        case CMD_HOST_SIDE_GET_MODULE_STATIC_DATA:
            upload_module_static_data();
            break;

        case CMD_HOST_SIDE_SET_MODULE_KERNEL_TYPE:
            adaps_set_module_kernel_type(pCmdData, rxDataLen);
            break;

        case CMD_HOST_SIDE_SET_WALKERROR_ENABLE:
            adaps_set_walkerror_enable(pCmdData, rxDataLen);
            break;

        case CMD_HOST_SIDE_SET_HISTOGRAM_DATA_REQ_POSITION:
            adaps_set_req_histogram_position(pCmdData, rxDataLen);
            break;

        case CMD_HOST_SIDE_SET_COLORMAP_RANGE_PARAM:
            adaps_set_colormap_range(pCmdData, rxDataLen);
            break;

        case CMD_HOST_SIDE_DOWNLOAD_ROI_SRAM_DATA:
            download_roisram_data(pCmdData, rxDataLen);
            break;

        case CMD_HOST_SIDE_DOWNLOAD_SPOT_WALKERROR_DATA:
            dump_buffer_data(pCmdData, "CMD_HOST_SIDE_DOWNLOAD_SPOT_WALKERROR_DATA", __LINE__);
            download_walkerror_data(pCmdData, rxDataLen);
            break;

        case CMD_HOST_SIDE_DOWNLOAD_SPOT_OFFSET_DATA:
            download_spotoffset_data(pCmdData, rxDataLen);
            break;

        case CMD_HOST_SIDE_DOWNLOAD_REF_DISTANCE_DATA:
            download_ref_distance_data(pCmdData, rxDataLen);
            break;

        case CMD_HOST_SIDE_DOWNLOAD_LENS_INTRINSIC_DATA:
            download_lens_intrinsic_data(pCmdData, rxDataLen);
            break;

        case CMD_HOST_SIDE_SET_DEVICE_REBOOT:
            adaps_request_device_reboot(pCmdData, rxDataLen);
            break;

        case CMD_HOST_SIDE_UPDATE_EEPROM_DATA:
            adaps_update_eeprom_data(pCmdData, rxDataLen);
            break;

        case CMD_HOST_SIDE_START_CAPTURE:
            adaps_start_capture(pCmdData, rxDataLen);
            break;

        case CMD_HOST_SIDE_STOP_CAPTURE:
            if (true != Utils::is_env_var_true(ENV_VAR_DEVELOP_DEBUGGING))
            {
                qApp->set_capture_req_from_host(false);
                emit stop_capture();
                DBG_NOTICE("------communication statistics------total txRawdataFrameCnt: %ld (%d fps), txDepth16FrameCnt: %ld (%d fps), txPointCloudFrameCnt: %ld (%d fps)---\n",
                    txRawdataFrameCnt, txRawdataFrame_fps, txDepth16FrameCnt, txDepth16Frame_fps, txPointCloudFrameCnt, txPointCloudFrame_fps);
                reset_data();
            }
            break;

        case CMD_HOST_SIDE_GET_SENSOR_REGISTER:
        case CMD_HOST_SIDE_GET_VCSLDRV_OP7020_REGISTER:
            if (NULL_POINTER == v4l2)
            {
                DBG_ERROR("v4l2 is NULL");
                return;
            }

            if (true == v4l2->Get_power_on_state())
            {
                read_device_register(pCmdData->cmd, pCmdData, rxDataLen);
            } else {
                LOG_WARN("cmd %u: sensor is power off.\n", pCmdData->cmd);
            }
            break;

        case CMD_HOST_SIDE_SET_SENSOR_REGISTER:
        case CMD_HOST_SIDE_SET_VCSLDRV_OP7020_REGISTER:
            if (NULL_POINTER == v4l2)
            {
                DBG_ERROR("v4l2 is NULL");
                return;
            }

            if (true == v4l2->Get_power_on_state())
            {
                write_device_register(pCmdData->cmd, pCmdData, rxDataLen);
            } else {
                LOG_WARN("cmd %u: sensor is power off.\n", pCmdData->cmd);
            }
            break;

        default:
            LOG_ERROR("Unknown swift_event_process: cmd %d.\n", pCmdData->cmd);
            break;
    }

    return;
}

void Host_Communication::adaps_sender_disconnected()
{
    LOG_WARN("sender is disconnected now.\n");

    if (true != Utils::is_env_var_true(ENV_VAR_DEVELOP_DEBUGGING))
    {
        qApp->set_capture_req_from_host(false);
        emit stop_capture();
        connected = false;
        reset_data();
    }

    return;
}

void Host_Communication::adaps_sender_connected()
{
    LOG_WARN("sender connect success.\n");

    connected = true;

    return;
}

int Host_Communication::adaps_sender_callback(SenderEventId_t id, void* arg_ptr, uint32_t arg_u32, ...)
{
    Host_Communication* obj = instance;
    //LOG_DEBUG("swift_sender_callback_server: id = %d.\n", id);

    switch (id)
    {
    case SENDER_EVT_CONNECTED:
        obj->adaps_sender_connected();
        break;

    case SENDER_EVT_DISCONNECTED:
        obj->adaps_sender_disconnected();
        break;

    case SENDER_EVT_RECEIVED_MSG:
        obj->adaps_event_process(arg_ptr, arg_u32);
        break;

    default:
        LOG_ERROR("Unknown sender evt:%d\n", id);
        break;
    }

    return 0;
}

int Host_Communication::adaps_sender_init()
{
    int ret = 0;
    sender_init_param_t init_param;

    sender_log_config(SENDER_LOG_WARN, (sender_log_output_func_t)SENDER_LOG_FUNC);

    memset(&init_param, 0, sizeof(init_param));

    init_param.host_or_device = SENDER_RUN_AS_DEVICE;
    init_param.tcp_port = TCP_SERVER_PORT;
    init_param.callback = adaps_sender_callback;

    ret = sender_init(&init_param);
    LOG_DEBUG("sender init ret %d, sender lib version: %s\n", ret, sender_get_version_str());
    if (ret != 0) {
        LOG_ERROR("sender init failed, ret %d\n", ret);
        return -1;
    }

    return ret;
}

#endif // !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)

