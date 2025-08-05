#include <mainwindow.h>
#include <globalapplication.h>

#include "FrameProcessThread.h"
#include "utils.h"

FrameProcessThread::FrameProcessThread()
{
    stopped = true;
    majorindex = -1;
    rgb_buffer = NULL_POINTER;
    confidence_map_buffer = NULL_POINTER;
    depth_buffer = NULL_POINTER;
#if 0 //defined(ENABLE_DYNAMICALLY_UPDATE_ROI_SRAM_CONTENT)
    merged_depth_buffer = NULL_POINTER;
    merged_frame_cnt = 0;
#endif
    outputed_frame_cnt = 0;
    v4l2 = NULL_POINTER;
    utils = NULL_POINTER;
    sns_param.sensor_type = qApp->get_sensor_type();
    sns_param.save_frame_cnt = qApp->get_save_cnt();
    skip_frame_decode = Utils::is_env_var_true(ENV_VAR_SKIP_FRAME_DECODE);
    dump_spot_statistics_times = Utils::get_env_var_intvalue(ENV_VAR_DUMP_SPOT_STATISTICS_TIMES);
    dump_ptm_frame_headinfo_times = Utils::get_env_var_intvalue(ENV_VAR_DUMP_PTM_FRAME_HEADINFO_TIMES);

#if defined(RUN_ON_EMBEDDED_LINUX)
    if (SENSOR_TYPE_DTOF == sns_param.sensor_type)
    {
        sns_param.work_mode = qApp->get_wk_mode();
        sns_param.env_type = qApp->get_environment_type();
        sns_param.measure_type = qApp->get_measurement_type();
        sns_param.framerate_type = qApp->get_framerate_type();
        sns_param.power_mode = qApp->get_power_mode();

        utils = new Utils();
        v4l2 = new V4L2(sns_param);
        v4l2->Get_frame_size_4_curr_wkmode(&sns_param.raw_width, &sns_param.raw_height, &sns_param.out_frm_width, &sns_param.out_frm_height);
        /// utils->GetPidTid(__FUNCTION__, __LINE__);
        DBG_INFO( "raw_width: %d raw_height: %d, env_type: %d, measure_type: %d, framerate_type: %d\n",
            sns_param.raw_width, sns_param.raw_height, sns_param.env_type, sns_param.measure_type, sns_param.framerate_type);
        adaps_dtof = new ADAPS_DTOF(sns_param);
    }
    else 
#endif
    {
        sns_param.work_mode = qApp->get_wk_mode();
        v4l2 = new V4L2(sns_param);
        v4l2->Get_frame_size_4_curr_wkmode(&sns_param.raw_width, &sns_param.raw_height, &sns_param.out_frm_width, &sns_param.out_frm_height);
    }
    qApp->register_v4l2_instance(v4l2);

#if defined(RUN_ON_EMBEDDED_LINUX) && !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
    qRegisterMetaType<frame_buffer_param_t>("frame_buffer_param_t");
    connect(v4l2, SIGNAL(rx_new_frame(unsigned int, void *, int, struct timeval, enum frame_data_type, int, frame_buffer_param_t)),
            this, SLOT(new_frame_handle(unsigned int, void *, int, struct timeval, enum frame_data_type, int, frame_buffer_param_t)), Qt::DirectConnection);
#else
    connect(v4l2, SIGNAL(rx_new_frame(unsigned int, void *, int, struct timeval, enum frame_data_type, int)),
            this, SLOT(new_frame_handle(unsigned int, void *, int, struct timeval, enum frame_data_type, int)), Qt::DirectConnection);
#endif

    qRegisterMetaType<status_params1>("status_params1");
    connect(v4l2, SIGNAL(update_info(status_params1)),  this, SLOT(info_update(status_params1)));
    connect(this, SIGNAL(threadLoopExit()), this, SLOT(onThreadLoopExit()));

    expected_md5_string = Utils::get_env_var_stringvalue(ENV_VAR_EXPECTED_FRAME_MD5SUM);
}

FrameProcessThread::~FrameProcessThread()
{
    if (NULL_POINTER != utils)
    {
        delete utils;
        utils = NULL_POINTER;
    }

#if defined(RUN_ON_EMBEDDED_LINUX)
    if (NULL_POINTER != depth_buffer)
    {
        free(depth_buffer);
        depth_buffer = NULL_POINTER;
    }

#if 0 //defined(ENABLE_DYNAMICALLY_UPDATE_ROI_SRAM_CONTENT)
    if (NULL_POINTER != merged_depth_buffer)
    {
        free(merged_depth_buffer);
        merged_depth_buffer = NULL_POINTER;
    }
#endif

    if (NULL_POINTER != adaps_dtof)
    {
        delete adaps_dtof;
        adaps_dtof = NULL_POINTER;
    }
#endif

    if (NULL_POINTER != rgb_buffer)
    {
        free(rgb_buffer);
        rgb_buffer = NULL_POINTER;
    }

    if (NULL_POINTER != confidence_map_buffer)
    {
        free(confidence_map_buffer);
        confidence_map_buffer = NULL_POINTER;
    }

    if (NULL_POINTER != v4l2)
    {
        delete v4l2;
        v4l2 = NULL_POINTER;
    }
}

void FrameProcessThread::save_depth_txt_file(void *frm_buf,unsigned int frm_sequence,int frm_len)
{
    Q_UNUSED(frm_sequence);
    QDateTime       localTime = QDateTime::currentDateTime();
    QString         currentTime = localTime.toString("yyyyMMddhhmmss");
    char *          LocalTimeStr = (char *) currentTime.toStdString().c_str();

    char path[50]={0};
    int16_t *p_temp=(int16_t*)frm_buf;

    sprintf(path,"%sframe%03d_%s_%d_depth16%s",DATA_SAVE_PATH,frm_sequence, LocalTimeStr, frm_len, ".txt");
    FILE*fp = NULL_POINTER;
    fp=fopen(path, "w+");
    if (fp == NULL_POINTER)
    {
        DBG_ERROR("fopen output file %s failed!\n",  path);
        return;
    }
    for (int i = 0; i < OUTPUT_HEIGHT_4_DTOF_SENSOR; i++)
    {
        int offset = i * OUTPUT_WIDTH_4_DTOF_SENSOR;
        for (int j = 0; j < OUTPUT_WIDTH_4_DTOF_SENSOR; j++)
        {
            fprintf(fp, "%6u ", (*(p_temp + offset + j))&0x1fff);  //do not printf high 3 bit confidence
        }
        fprintf(fp, "\n");
    }
    fflush(fp);
    fclose(fp);
    DBG_INFO("Save depth file %s success!\n",  path);

}

bool FrameProcessThread::save_frame(unsigned int frm_sequence, void *frm_buf, int buf_size, int frm_w, int frm_h, struct timeval frm_timestamp, enum frame_data_type ftype)
{
    const char extName[FDATA_TYPE_COUNT][24]   = {
                                    ".nv12",
                                    ".yuyv",
                                    ".rgb888",
                                    ".raw_grayscale",
                                    ".raw_depth",
                                    ".decoded_grayscale",
                                    ".decoded_depth16"
                                };

    Q_UNUSED(frm_timestamp);

    QDateTime       localTime = QDateTime::currentDateTime();
    QString         currentTime = localTime.toString("yyyyMMddhhmmss");
    char *          LocalTimeStr = (char *) currentTime.toStdString().c_str();
    char *          filename = new char[128];

    sprintf(filename, "%sframe%03d_%dx%d_%s_%d%s", DATA_SAVE_PATH, frm_sequence, frm_w, frm_h,LocalTimeStr, buf_size, extName[ftype]);
    utils->save_binary_file(filename, frm_buf, 
        buf_size,
        __FUNCTION__,
        __LINE__
        );
    delete[] filename;

    return true;
}

bool FrameProcessThread::info_update(status_params1 param1)
{
    status_params2 param2;

    param2.sensor_type = sns_param.sensor_type;
    param2.mipi_rx_fps = param1.mipi_rx_fps;
    param2.streamed_time_us = param1.streamed_time_us;
    param2.work_mode = sns_param.work_mode;
#if defined(RUN_ON_EMBEDDED_LINUX)
    param2.curr_temperature = param1.curr_temperature;
    param2.curr_exp_vop_abs = param1.curr_exp_vop_abs;
    param2.curr_exp_pvdd = param1.curr_exp_pvdd;
    param2.env_type = sns_param.env_type;
    param2.measure_type = sns_param.measure_type;
    param2.curr_power_mode = sns_param.power_mode;
#endif

    emit update_runtime_display(param2);

    return true;
}

bool FrameProcessThread::new_frame_handle(
    unsigned int frm_sequence, 
    void *frm_rawdata, 
    int buf_len, 
    struct timeval frm_timestamp, 
    enum frame_data_type ftype, 
    int total_bytes_per_line
#if defined(RUN_ON_EMBEDDED_LINUX) && !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
    , frame_buffer_param_t frmBufParam
#endif
    )
{
    int decodeRet = 0;
#if defined(RUN_ON_EMBEDDED_LINUX)
    static int run_times = 0;
#endif

    Q_UNUSED(frm_sequence);
    Q_UNUSED(frm_timestamp);
    Q_UNUSED(buf_len);
    Q_UNUSED(total_bytes_per_line);

#if 0
    DBG_INFO("skipFrameProcess:%d, frm_sequence:%d, buf_len:%d", skip_frame_decode, frm_sequence, buf_len);
#endif

    switch (ftype) {
#if defined(RUN_ON_EMBEDDED_LINUX)
        case FDATA_TYPE_DTOF_RAW_GRAYSCALE:
            if (adaps_dtof)
            {
                run_times++;

                if (sns_param.save_frame_cnt > 0)
                {
                    if (true == Utils::is_env_var_true(ENV_VAR_SAVE_FRAME_ENABLE))
                    {
                        save_frame(frm_sequence,frm_rawdata,buf_len,
                            sns_param.raw_width, sns_param.raw_height,
                            frm_timestamp, FDATA_TYPE_DTOF_RAW_GRAYSCALE);
                    }
                }

                if (skip_frame_decode)
                {
                    int i;
                    decodeRet = 0;

                    for (i = 0; i < sns_param.out_frm_width*sns_param.out_frm_height; i++)
                    {
                        rgb_buffer[i*3] = 0x0;     // R component, fill with min value 0x0
                        rgb_buffer[i*3 + 1] = 0x0;  // G component, fill with min value 0x00
                        rgb_buffer[i*3 + 2] = 0xFF;  // B component, fill with max value 0xFF
                    }
                }
                else {
                    decodeRet = adaps_dtof->dtof_frame_decode(frm_sequence, (unsigned char *)frm_rawdata, buf_len, depth_buffer, sns_param.work_mode);
                    if (0 == decodeRet)
                    {
#if !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
                        Host_Communication *host_comm = Host_Communication::getInstance();
                        if (host_comm)
                        {
                            frmBufParam.data_type = FRAME_DECODED_DEPTH16;
                            frmBufParam.frm_width = sns_param.out_frm_width;
                            frmBufParam.frm_height = sns_param.out_frm_height;
                            frmBufParam.padding_bytes_per_line = 0;

                            host_comm->report_frame_depth16_data(depth_buffer, depth_buffer_size, &frmBufParam);
                        }
#endif

                        if (sns_param.save_frame_cnt > 0)
                        {
                            if (true == Utils::is_env_var_true(ENV_VAR_SAVE_FRAME_ENABLE))
                            {
                                save_frame(frm_sequence,depth_buffer,depth_buffer_size,
                                    sns_param.out_frm_width, sns_param.out_frm_height,
                                    frm_timestamp, FDATA_TYPE_DTOF_DECODED_GRAYSCALE);
                            }
                        }
#if !defined(CONSOLE_APP_WITHOUT_GUI)
                        adaps_dtof->ConvertGreyscaleToColoredMap(depth_buffer,rgb_buffer, sns_param.out_frm_width,sns_param.out_frm_height);
                        if (sns_param.save_frame_cnt > 0)
                        {
                            if (true == Utils::is_env_var_true(ENV_VAR_SAVE_FRAME_ENABLE))
                            {
                                save_frame(frm_sequence,rgb_buffer,sns_param.out_frm_width*sns_param.out_frm_height*RGB_IMAGE_CHANEL,
                                    sns_param.out_frm_width, sns_param.out_frm_height,
                                    frm_timestamp, FDATA_TYPE_RGB888);
                            }
                        }
#endif
                    }
                    else {
                        DBG_ERROR("dtof_frame_decode() return %d , save_frame_cnt: %d...", decodeRet, sns_param.save_frame_cnt);
                    }
                }
            }
            break;

        case FDATA_TYPE_DTOF_RAW_DEPTH:
            if (adaps_dtof)
            {
                if (frm_sequence < dump_ptm_frame_headinfo_times && 0 != dump_ptm_frame_headinfo_times)
                {
                    adaps_dtof->dump_frame_headinfo(frm_sequence, (unsigned char *)frm_rawdata, buf_len, sns_param.work_mode);
                }

                if (NULL_POINTER != expected_md5_string)
                {
                    // swift test pattern output, the first 2 lines/packets have some variable values, so I'm ignoring them.
                    // On rockchip platform:
                    //frame raw size: 4104 X 32, bits_per_pixel: 8, payload_bytes_per_line: 4104, total_bytes_per_line: 4352, padding_bytes_per_line 248, frame_buffer_size: 139264
                    //
                    // export expected_frame_md5sum="85f24805d05ed40d63426bc193094e84" for spot register setting
                    // export expected_frame_md5sum="5b3cceee5be473e943817e9800e70a40" for flood register setting
                    // echo 0x8 > /sys/kernel/debug/adaps/dbg_ctrl
                    utils->MD5Check4Buffer(
                        static_cast<unsigned char*>(frm_rawdata) + total_bytes_per_line * 2,
                        buf_len - total_bytes_per_line *2,
                        (const char *) expected_md5_string,
                        __FUNCTION__,
                        __LINE__
                        );
                }

                if (sns_param.save_frame_cnt > 0)
                {
                    if (true == Utils::is_env_var_true(ENV_VAR_SAVE_FRAME_ENABLE))
                    {
#if 0
                        save_frame(frm_sequence,frm_rawdata,buf_len,
                            sns_param.raw_width, sns_param.raw_height,
                            frm_timestamp, FDATA_TYPE_DTOF_RAW_DEPTH);
#else
                        // swift test pattern output, the first 2 lines/packets have some variable values, so I'm ignoring them.
                        save_frame(frm_sequence,
                            static_cast<unsigned char*>(frm_rawdata) + total_bytes_per_line * 2,
                            buf_len - total_bytes_per_line *2,
                            sns_param.raw_width, sns_param.raw_height,
                            frm_timestamp, FDATA_TYPE_DTOF_RAW_DEPTH);
#endif
                    }
                }

                if (skip_frame_decode)
                {
                    int i;
                    decodeRet = 0;

                    for (i = 0; i < sns_param.out_frm_width*sns_param.out_frm_height; i++)
                    {
                        rgb_buffer[i*3] = 0xFF;     // R component, fill with max value 0xFF
                        rgb_buffer[i*3 + 1] = 0x0;  // G component, fill with min value 0x00
                        rgb_buffer[i*3 + 2] = 0x0;  // B component, fill with min value 0x00
                    }
                }
                else {
                    decodeRet = adaps_dtof->dtof_frame_decode(frm_sequence, (unsigned char *)frm_rawdata, buf_len, depth_buffer, sns_param.work_mode);

#if 0 //defined(ENABLE_DYNAMICALLY_UPDATE_ROI_SRAM_CONTENT)
                    if (frm_sequence < 80 && frm_sequence > 34)
                    {
                        adaps_dtof->dumpSpotCount(depth_buffer, sns_param.out_frm_width, sns_param.out_frm_height, frm_sequence, merged_frame_cnt, decodeRet, __LINE__);
                    }

                    int new_added_spot_count = adaps_dtof->DepthBufferMerge(merged_depth_buffer, depth_buffer, sns_param.out_frm_width, sns_param.out_frm_height);
                    if (frm_sequence < 80 && frm_sequence > 34)
                    {
                        adaps_dtof->dumpSpotCount(merged_depth_buffer, sns_param.out_frm_width, sns_param.out_frm_height, frm_sequence, merged_frame_cnt, decodeRet, __LINE__);
                    }

                    if (0 == decodeRet)
                    {
                        Host_Communication *host_comm = Host_Communication::getInstance();

                        merged_frame_cnt++;
                        adaps_dtof->depthMapDump(merged_depth_buffer, sns_param.out_frm_width, sns_param.out_frm_height, merged_frame_cnt, __LINE__);

                        if (host_comm)
                        {
                            frmBufParam.data_type = FRAME_DECODED_DEPTH16;
                            frmBufParam.frm_width = sns_param.out_frm_width;
                            frmBufParam.frm_height = sns_param.out_frm_height;
                            frmBufParam.padding_bytes_per_line = 0;

                            host_comm->report_frame_depth16_data(merged_depth_buffer, depth_buffer_size, &frmBufParam);
                        }

                        if (sns_param.save_frame_cnt > 0)
                        {
                            if (true == Utils::is_env_var_true(ENV_VAR_SAVE_FRAME_ENABLE))
                            {
                                save_frame(frm_sequence, merged_depth_buffer, depth_buffer_size,
                                    sns_param.out_frm_width, sns_param.out_frm_height,
                                    frm_timestamp, FDATA_TYPE_DTOF_DECODED_DEPTH16);
                            }

                            if (Utils::is_env_var_true(ENV_VAR_SAVE_DEPTH_TXT_ENABLE))
                            {
                                save_depth_txt_file(merged_depth_buffer, frm_sequence, depth_buffer_size);
                            }
                        }
#if !defined(CONSOLE_APP_WITHOUT_GUI)
                        adaps_dtof->ConvertDepthToColoredMap(merged_depth_buffer, rgb_buffer, confidence_map_buffer, sns_param.out_frm_width, sns_param.out_frm_height);
                        if (sns_param.save_frame_cnt > 0)
                        {
                            if (true == Utils::is_env_var_true(ENV_VAR_SAVE_FRAME_ENABLE))
                            {
                                save_frame(frm_sequence,rgb_buffer,sns_param.out_frm_width*sns_param.out_frm_height*RGB_IMAGE_CHANEL,
                                    sns_param.out_frm_width, sns_param.out_frm_height,
                                    frm_timestamp, FDATA_TYPE_RGB888);
                            }
                        }
#endif
                    }
                    else {
                        //DBG_ERROR("dtof_frame_decode() return %d , errno: %s (%d), save_frame_cnt: %d...", decodeRet, strerror(errno), errno, sns_param.save_frame_cnt);
                    }
#else
                    if (0 == decodeRet)
                    {
#if !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
                        Host_Communication *host_comm = Host_Communication::getInstance();
#endif

                        outputed_frame_cnt++;
                        if (Utils::is_env_var_true(ENV_VAR_DEPTH16_FILE_REPLAY_ENABLE))
                        {
                            if (true == utils->is_replay_data_exist())
                            {
                                QByteArray buffer = utils->loadNextFileToBuffer();
                                if (buffer.isEmpty()) {
                                    DBG_ERROR("Fail to loadNextFileToBuffer, return empty, depth_buffer_size: %d!", depth_buffer_size);
                                }
                                else {
                                    std::string stdStr = buffer.toStdString();
                                    memcpy(depth_buffer, (void *) stdStr.c_str(), depth_buffer_size);
                                    DBG_NOTICE("loadNextFileToBuffer() return success, depth_buffer_size: %d!", depth_buffer_size);
                                    //depth_buffer = (void *) stdStr.c_str();
                                }
                            }
                        }

                        if (frm_sequence < dump_spot_statistics_times && 0 != dump_spot_statistics_times)
                        {
                            adaps_dtof->dumpSpotCount(depth_buffer, sns_param.out_frm_width, sns_param.out_frm_height, frm_sequence, outputed_frame_cnt, decodeRet, __LINE__);
                        }
                        adaps_dtof->depthMapDump(depth_buffer, sns_param.out_frm_width, sns_param.out_frm_height, outputed_frame_cnt, __LINE__);

#if !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
                        if (host_comm)
                        {
                            frmBufParam.data_type = FRAME_DECODED_DEPTH16;
                            frmBufParam.frm_width = sns_param.out_frm_width;
                            frmBufParam.frm_height = sns_param.out_frm_height;
                            frmBufParam.padding_bytes_per_line = 0;

                            host_comm->report_frame_depth16_data(depth_buffer, depth_buffer_size, &frmBufParam);
                        }
#endif

                        if (sns_param.save_frame_cnt > 0)
                        {
                            if (true == Utils::is_env_var_true(ENV_VAR_SAVE_FRAME_ENABLE))
                            {
                                save_frame(frm_sequence, depth_buffer, depth_buffer_size,
                                    sns_param.out_frm_width, sns_param.out_frm_height,
                                    frm_timestamp, FDATA_TYPE_DTOF_DECODED_DEPTH16);
                            }

                            if (Utils::is_env_var_true(ENV_VAR_SAVE_DEPTH_TXT_ENABLE))
                            {
                                save_depth_txt_file(depth_buffer, frm_sequence, sns_param.out_frm_width*sns_param.out_frm_height*sizeof(u16));
                            }
                        }
#if !defined(CONSOLE_APP_WITHOUT_GUI)
                        adaps_dtof->ConvertDepthToColoredMap(depth_buffer, rgb_buffer, confidence_map_buffer, sns_param.out_frm_width, sns_param.out_frm_height);
                        if (sns_param.save_frame_cnt > 0)
                        {
                            if (true == Utils::is_env_var_true(ENV_VAR_SAVE_FRAME_ENABLE))
                            {
                                save_frame(frm_sequence,rgb_buffer,sns_param.out_frm_width*sns_param.out_frm_height*RGB_IMAGE_CHANEL,
                                    sns_param.out_frm_width, sns_param.out_frm_height,
                                    frm_timestamp, FDATA_TYPE_RGB888);
                            }
                        }
#endif
                    }
                    else {
                        //DBG_ERROR("dtof_frame_decode() return %d , frm_sequence: %d, adaps_dtof: %p...", decodeRet, frm_sequence, adaps_dtof);
                    }
#endif
                }
            }
            else {
                DBG_ERROR("adaps_dtof is NULL");
            }
            break;
#endif

        case FDATA_TYPE_YUYV:
            if (v4l2)
            {
                utils->yuyv_2_rgb((unsigned char *)frm_rawdata,rgb_buffer,sns_param.out_frm_width,sns_param.out_frm_height);
            }
            break;

        case FDATA_TYPE_NV12:
            if (v4l2)
            {
                utils->nv12_2_rgb((unsigned char *)frm_rawdata,rgb_buffer,sns_param.out_frm_width,sns_param.out_frm_height);
            }
        break;

        default:
            DBG_ERROR("Unsupported input frame type (%d)...", ftype);
            decodeRet = -EINVAL;
            break;
    }
    if (0 == decodeRet)
    {
#if 0 //defined(ENABLE_DYNAMICALLY_UPDATE_ROI_SRAM_CONTENT)
        if (NULL_POINTER != merged_depth_buffer)
        {
            memset(merged_depth_buffer, 0, depth_buffer_size);
        }
#endif
        if (NULL_POINTER != depth_buffer)
        {
            memset(depth_buffer, 0, depth_buffer_size);
        }
#if !defined(CONSOLE_APP_WITHOUT_GUI)
        QImage img = QImage(rgb_buffer, sns_param.out_frm_width, sns_param.out_frm_height, sns_param.out_frm_width*RGB_IMAGE_CHANEL,QImage::Format_RGB888);
        if (FDATA_TYPE_DTOF_RAW_DEPTH == ftype)
        {
            QImage img4confidence = QImage(confidence_map_buffer, sns_param.out_frm_width, sns_param.out_frm_height, sns_param.out_frm_width*RGB_IMAGE_CHANEL,QImage::Format_RGB888);
            emit newFrameReady4Display(img, img4confidence);
        }
        else {
            QImage img4confidence;
            emit newFrameReady4Display(img, img4confidence);
        }
#endif
    }
    if (sns_param.save_frame_cnt > 0)
    {
        sns_param.save_frame_cnt--;
    }

    return true;
}

void FrameProcessThread::onThreadLoopExit()
{
    if (v4l2)
    {
        v4l2->Stop_streaming();
        v4l2->V4l2_close();
    }
    if (SENSOR_TYPE_DTOF == sns_param.sensor_type)
    {
#if defined(RUN_ON_EMBEDDED_LINUX)
        if (adaps_dtof)
        {
            adaps_dtof->adaps_dtof_release();
        }
#endif
        if (NULL_POINTER != depth_buffer)
        {
            free(depth_buffer);
            depth_buffer = NULL_POINTER;
        }
#if 0 //defined(ENABLE_DYNAMICALLY_UPDATE_ROI_SRAM_CONTENT)
        if (NULL_POINTER != merged_depth_buffer)
        {
            free(merged_depth_buffer);
            merged_depth_buffer = NULL_POINTER;
        }
#endif

        if (NULL_POINTER != confidence_map_buffer)
        {
            free(confidence_map_buffer);
            confidence_map_buffer = NULL_POINTER;
        }
    }
    if (NULL_POINTER != rgb_buffer)
    {
        free(rgb_buffer);
        rgb_buffer = NULL_POINTER;
    }

    emit threadEnd(stop_req_code);
}

void FrameProcessThread::stop(int stop_request_code)
{
    stop_req_code = stop_request_code;
    if(!stopped)
    {
        stopped = true;
    }
    else {
        emit threadLoopExit();
    }
}

bool FrameProcessThread::isSleeping()
{
    return sleeping;
}

int FrameProcessThread::init(int index)
{
    int ret = 0;
    int ret_4_start_stream = 0;
    ret = v4l2->V4l2_initilize();
    if (ret < 0)
    {
        DBG_ERROR("Fail to v4l2->V4l2_initilize(),ret:%d, errno: %s (%d)...", ret,
            strerror(errno), errno);
        return ret;
    }
    else {
        ret_4_start_stream = v4l2->Start_streaming(); // need this before adaps_dtof->adaps_dtof_initilize() to get the real exposure_period ptm_fine_exposure_value in initParams()
        if (ret_4_start_stream < 0)
        {
            DBG_ERROR("Fail to v4l2->V4l2_initilize(),ret:%d, errno: %s (%d)...", ret_4_start_stream,
                strerror(errno), errno);
            return ret_4_start_stream;
        }

        if (SENSOR_TYPE_DTOF == sns_param.sensor_type)
        {
#if defined(RUN_ON_EMBEDDED_LINUX)
            if (NULL_POINTER == adaps_dtof)
            {
                DBG_ERROR("adaps_dtof is NULL" );
                return 0 - __LINE__;
            }

            ret = adaps_dtof->adaps_dtof_initilize();
            if (0 > ret)
            {
                DBG_ERROR("Fail to adaps_dtof->adaps_dtof_initilize(),ret:%d, errno: %s (%d)...", 
                    ret, strerror(errno), errno);
                return 0 - __LINE__;
            }
            else {
                depth_buffer_size = sizeof(u16)*sns_param.out_frm_width*sns_param.out_frm_height;

                depth_buffer = (u16 *)malloc(depth_buffer_size);
                if (NULL_POINTER != depth_buffer)
                {
                    memset(depth_buffer, 0, depth_buffer_size);
                }

#if 0 //defined(ENABLE_DYNAMICALLY_UPDATE_ROI_SRAM_CONTENT)
                merged_depth_buffer = (u16 *)malloc(depth_buffer_size);
                if (NULL_POINTER != merged_depth_buffer)
                {
                    memset(merged_depth_buffer, 0, depth_buffer_size);
                }
#endif
            }
            confidence_map_buffer = (unsigned char *)malloc(sizeof(unsigned char)*sns_param.out_frm_width*sns_param.out_frm_height*RGB_IMAGE_CHANEL);
            memset(confidence_map_buffer, 0, sns_param.out_frm_width*sns_param.out_frm_height*RGB_IMAGE_CHANEL);
#endif
        }
        rgb_buffer = (unsigned char *)malloc(sizeof(unsigned char)*sns_param.out_frm_width*sns_param.out_frm_height*RGB_IMAGE_CHANEL);
        memset(rgb_buffer, 0, sizeof(unsigned char)*sns_param.out_frm_width*sns_param.out_frm_height*RGB_IMAGE_CHANEL);
        if (0 == ret_4_start_stream)
        {
            stopped = false;
            majorindex = index;
        }
    }

    return ret;
}

void FrameProcessThread::run()
{
    int ret = 0;
    /// static int run_times = 0;

    if(majorindex != -1)
    {
        while(!stopped)
        {
            if (this->isInterruptionRequested()) {
                 DBG_NOTICE("Thread is interrupted, cleaning up...");
                 break;
             }

            if(!stopped)
            {
                /// if (run_times < 1)
                /// {
                ///     utils->GetPidTid(__FUNCTION__, __LINE__);
                /// }
                ret=v4l2->Capture_frame();
                sleeping = true;
                QThread::usleep(FRAME_PROCESS_THREAD_INTERVAL_US);
                sleeping = false;
            }

            if(ret < 0)
            {
                stopped = true;
            }
            else {
                if ((0 != qApp->get_save_cnt()) && (0 == sns_param.save_frame_cnt)) // if already capture expected frames, try to quit.
                {
                    stop(STOP_REQUEST_STOP);
                    //break;
                }
            }

            /// run_times++;
        }

        emit threadLoopExit();
    }
}

