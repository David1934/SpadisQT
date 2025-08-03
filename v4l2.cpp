#include <sys/sysmacros.h>
#include <linux/v4l2-subdev.h>

#include "v4l2.h"
#include "utils.h"
#include "misc_device.h"
#include <globalapplication.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

V4L2::V4L2(struct sensor_params params)
{
#if defined(RUN_ON_EMBEDDED_LINUX)
    fd_4_dtof = 0;
#if !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
    host_comm = Host_Communication::getInstance();
#endif
    utils = new Utils();
    p_misc_device = NULL_POINTER;
#endif
    buffers = NULL_POINTER;
    sensordata = NULL_POINTER;
    fd = 0;
    firstFrameTimeUsec = 0;
    rxFrameCnt = 0;
    mipi_rx_fps = 0;
    streamed_timeUs = 0;
    stream_on = false;
    power_on = false;
    init();
    memcpy(&snr_param, &params, sizeof(struct sensor_params));

    if (NULL_POINTER != sensordata[params.work_mode].sensor_subdev)
    {
        memcpy(sensor_sd_name, sensordata[params.work_mode].sensor_subdev, DEV_NODE_LEN);
    }
    else {
        memset(sensor_sd_name, 0, DEV_NODE_LEN);
    }
    memcpy(media_dev, sensordata[params.work_mode].media_devnode, DEV_NODE_LEN);
    memcpy(video_dev, sensordata[params.work_mode].video_devnode, DEV_NODE_LEN);
    snr_param.raw_width = sensordata[params.work_mode].raw_w;
    snr_param.raw_height = sensordata[params.work_mode].raw_h;
    pixel_format = sensordata[params.work_mode].pixfmt;
    frame_buffer_count = sensordata[params.work_mode].frm_buf_cnt;
    snr_param.sensor_type = sensordata[params.work_mode].stype;
    frm_type = sensordata[params.work_mode].ftype;
    snr_param.out_frm_width = sensordata[params.work_mode].out_frm_width;
    snr_param.out_frm_height = sensordata[params.work_mode].out_frm_height;

    DBG_INFO("sensor_sd_name: %s", sensor_sd_name);
    DBG_INFO("media_dev: %s", media_dev);
    DBG_INFO("video_dev: %s", video_dev);
    DBG_INFO("preset width: %d", snr_param.raw_width);
    DBG_INFO("preset height: %d", snr_param.raw_height);
    DBG_INFO("preset pixel_format: 0x%x", pixel_format);
    DBG_INFO("preset frame_buffer_count: %d", frame_buffer_count);
    DBG_INFO("preset sensor_type: %d", snr_param.sensor_type);
}


V4L2::~V4L2()
{
#if defined(RUN_ON_EMBEDDED_LINUX)
    if (NULL_POINTER != utils)
    {
        delete utils;
        utils = NULL_POINTER;
    }
#endif

    if (NULL_POINTER != sensordata)
    {
        free(sensordata);
        sensordata = NULL_POINTER;
    }
}

int V4L2::init()
{
    if (NULL_POINTER == sensordata)
    {
        sensordata = (struct sensor_data *)malloc(sizeof(struct sensor_data)*WK_COUNT);
    }

    sensordata[WK_DTOF_PHR] = {
        ENTITY_NAME_4_DTOF_SENSOR,
        MEDIA_DEVNAME_4_DTOF_SENSOR,
        VIDEO_DEV_4_DTOF_SENSOR,
        1032,
        MIPI_RAW_HEIGHT_4_DTOF_SENSOR,
        OUTPUT_WIDTH_4_DTOF_SENSOR,
        OUTPUT_HEIGHT_4_DTOF_SENSOR,
        PIXELFORMAT_4_DTOF_SENSOR,
        BUFFER_COUNT_4_DTOF_SENSOR,
        SENSOR_TYPE_DTOF,
        FDATA_TYPE_DTOF_RAW_DEPTH
    };

    sensordata[WK_DTOF_PCM] = {
        ENTITY_NAME_4_DTOF_SENSOR,
        MEDIA_DEVNAME_4_DTOF_SENSOR,
        VIDEO_DEV_4_DTOF_SENSOR,
        2560,
        MIPI_RAW_HEIGHT_4_DTOF_SENSOR,
        OUTPUT_WIDTH_4_DTOF_SENSOR,
        OUTPUT_HEIGHT_4_DTOF_SENSOR,
        PIXELFORMAT_4_DTOF_SENSOR,
        BUFFER_COUNT_4_DTOF_SENSOR,
        SENSOR_TYPE_DTOF,
        FDATA_TYPE_DTOF_RAW_GRAYSCALE
    };

    sensordata[WK_DTOF_FHR] = {
        ENTITY_NAME_4_DTOF_SENSOR,
        MEDIA_DEVNAME_4_DTOF_SENSOR,
        VIDEO_DEV_4_DTOF_SENSOR,
        4104,
        MIPI_RAW_HEIGHT_4_DTOF_SENSOR,
        OUTPUT_WIDTH_4_DTOF_SENSOR,
        OUTPUT_HEIGHT_4_DTOF_SENSOR,
        PIXELFORMAT_4_DTOF_SENSOR,
        BUFFER_COUNT_4_DTOF_SENSOR,
        SENSOR_TYPE_DTOF,
        FDATA_TYPE_DTOF_RAW_DEPTH
    };

    sensordata[WK_RGB_NV12] = {
        NULL_POINTER,
        MEDIA_DEVNAME_4_RGB_SENSOR,
        VIDEO_DEV_4_RGB_RK3588,
        640,
        480,
        640,
        480,
        V4L2_PIX_FMT_NV12,
        BUFFER_COUNT_4_RGB_SENSOR,
        SENSOR_TYPE_RGB,
        FDATA_TYPE_NV12
    };

    sensordata[WK_RGB_YUYV] = {
        NULL_POINTER,
        MEDIA_DEVNAME_4_RGB_SENSOR,
        VIDEO_DEV_4_RGB_SENSOR,
        640,
        480,
        640,
        480,
        V4L2_PIX_FMT_YUYV,
        BUFFER_COUNT_4_RGB_SENSOR,
        SENSOR_TYPE_RGB,
        FDATA_TYPE_YUYV
    };

    return 0;
}

bool V4L2::get_power_on_state()
{
    return power_on;
}

bool V4L2::get_stream_on_state()
{
    return stream_on;
}

int V4L2::get_devnode_from_sysfs(struct media_entity_desc *entity_desc, char *p_devname)
{
    struct stat devstat;
    char devname[32];
    char sysname[32];
    char target[1024];
    char *p;
    int ret;

    sprintf(sysname, "/sys/dev/char/%u:%u", entity_desc->v4l.major,
        entity_desc->v4l.minor);
    ret = readlink(sysname, target, sizeof(target) - 1);
    if (ret < 0)
        return -errno;

    target[ret] = '\0';
    p = strrchr(target, '/');
    if (p == NULL_POINTER)
        return -EINVAL;

    sprintf(devname, "/dev/%s", p + 1);
    ret = stat(devname, &devstat);
    if (ret < 0)
        return -errno;

    /* Sanity check: udev might have reordered the device nodes.
     * Make sure the major/minor match. We should really use
     * libudev.
     */
    if (major(devstat.st_rdev) == entity_desc->v4l.major &&
        minor(devstat.st_rdev) == entity_desc->v4l.minor)
        strcpy(p_devname, devname);

    return 0;
}

int V4L2::get_subdev_node_4_sensor()
{
    struct media_entity_desc entity_desc;
    uint32_t id = 0;
    int ret = -EINVAL;
    char cur_devnode[DEV_NODE_LEN];
    int fp;

    fp = open(media_dev, O_RDWR);
    if (fp < 0)
    {
        DBG_ERROR("Fail to open media device %s, errno: %s (%d)...", 
            media_dev, strerror(errno), errno);
        return 0 - __LINE__;
    }

    DBG_INFO("searching sensor %s...",sensor_sd_name);
    DBG_INFO("enum entities...");
    DBG_INFO("major:minor   id      entity_name       device node");
    DBG_INFO("-------------------------------------------------------");

    memset(&entity_desc, 0, sizeof(entity_desc));
    while(1)
    {
        entity_desc.id = id|MEDIA_ENT_ID_FLAG_NEXT;
        ret = ioctl(fp, MEDIA_IOC_ENUM_ENTITIES, &entity_desc);
        if(ret)
        {
            break;
        }

        id = entity_desc.id;
        get_devnode_from_sysfs(&entity_desc,cur_devnode);
        DBG_INFO("   %2d:%2d      %2d    %16s   %s", entity_desc.v4l.major, entity_desc.v4l.minor, id,entity_desc.name,cur_devnode);
        if(strcmp(sensor_sd_name, entity_desc.name) == 0)
        {
#if defined(RUN_ON_EMBEDDED_LINUX)
            strcpy(sd_devnode_4_dtof, cur_devnode);
            DBG_INFO("sd_devnode_4_dtof: %s", sd_devnode_4_dtof);
#endif
            ret = 0;
            break;
        }
    }

    return ret;
}

#if defined(RUN_ON_EMBEDDED_LINUX)
int V4L2::set_param_4_sensor_sub_device(int raw_w_4_curr_wkmode, int raw_h_4_curr_wkmode)
{
    int ret = 0;

    struct v4l2_subdev_format sensorFmt;

    memset(&sensorFmt, 0, sizeof(sensorFmt));
    sensorFmt.pad           = 0;
    sensorFmt.which         = V4L2_SUBDEV_FORMAT_ACTIVE;
    sensorFmt.format.width  = raw_w_4_curr_wkmode;
    sensorFmt.format.height = raw_h_4_curr_wkmode;
    DBG_INFO("--VIDIOC_SUBDEV_S_FMT--fd_4_dtof:%d, sd_devnode_4_dtof:%s, width:%d, height:%d", fd_4_dtof, sd_devnode_4_dtof, raw_w_4_curr_wkmode, raw_h_4_curr_wkmode);

    ret = ioctl(fd_4_dtof, VIDIOC_SUBDEV_S_FMT, &sensorFmt);
    if (-1 == ret) {
        DBG_ERROR("Fail to set format for dtof sensor sub device, errno: %s (%d)...", 
               strerror(errno), errno);
    }

    return ret;
}
#endif

bool V4L2::alloc_buffers(void)
{
    unsigned int i;

    req_bufs.count           = frame_buffer_count;
    req_bufs.type            = buf_type;
    req_bufs.memory          = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req_bufs) == -1) {
        DBG_ERROR("Fail to request buffer , buf_type:%d errno: %s (%d)...", 
            buf_type, strerror(errno), errno);
        return false;
    }

    buffers             = (struct buffer_s *) malloc(req_bufs.count * sizeof(struct buffer_s));

    if (!buffers) {
        DBG_ERROR("Out of memory");
        return false;
    }

    for (i = 0; i < req_bufs.count; i++)
    {
        struct v4l2_buffer	v4l2_buf;
        struct v4l2_plane v4l2_planes[FMT_NUM_PLANES];
        CLEAR(v4l2_planes);
        
        v4l2_buf.type = buf_type;
        v4l2_buf.memory = V4L2_MEMORY_MMAP;
        v4l2_buf.index          = i;
        
        if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == buf_type) {
            v4l2_buf.m.planes = v4l2_planes;
            v4l2_buf.length = FMT_NUM_PLANES;
        }

        if (-1 == ioctl (fd, VIDIOC_QUERYBUF, &v4l2_buf)) {
            DBG_ERROR("Fail to query buffer, errno: %s (%d)...", 
                strerror(errno), errno);
            return false;
        }
        
        if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == buf_type) {
            buffers[i].length = v4l2_buf.m.planes[0].length;
            buffers[i].start = mmap(NULL_POINTER /* start anywhere */,
                     v4l2_buf.m.planes[0].length,
                     PROT_READ | PROT_WRITE /* required */,
                     MAP_SHARED /* recommended */,
                     fd, v4l2_buf.m.planes[0].m.mem_offset);
        } else {
            buffers[i].length = v4l2_buf.length;
            buffers[i].start = mmap (NULL_POINTER, v4l2_buf.length,
                                      PROT_READ | PROT_WRITE,
                                      MAP_SHARED,
                                      fd, v4l2_buf.m.offset);
        }

        if (MAP_FAILED == buffers[i].start) {
            DBG_ERROR("Fail to mmap, errno: %s (%d)...", 
                strerror(errno), errno);
            return false;
        }
        DBG_INFO("buffer[%d].start: %p, length: %d", i, buffers[i].start, buffers[i].length);

        v4l2_buf.type = buf_type;
        v4l2_buf.memory = V4L2_MEMORY_MMAP;
        v4l2_buf.index = i;
        
        if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == buf_type) {
            v4l2_buf.m.planes = v4l2_planes;
            v4l2_buf.length = FMT_NUM_PLANES;
        }
        if (-1 == ioctl(fd, VIDIOC_QBUF, &v4l2_buf)) {
            DBG_ERROR("Fail to queue buffer, errno: %s (%d)...", 
                strerror(errno), errno);
            return false;
        }

    }

    return true;
}

void V4L2::free_buffers(void)
{
    unsigned int    i;

    if (NULL_POINTER != buffers)
    {
        for (i = 0; i < req_bufs.count; ++i) {
            if (-1 == munmap(buffers[i].start, buffers[i].length)) {
                DBG_ERROR("Fail to munmap, errno: %s (%d)...",
                    strerror(errno), errno);
                return;
            }
        }

        free(buffers);
        buffers = NULL_POINTER;
    }
}

UINT64 V4L2::timestamp_convert_from_timeval_to_us(struct timeval timestamp)
{
    return timestamp.tv_sec * 1000000LL + timestamp.tv_usec;
}

int V4L2::V4l2_initilize(void)
{
    struct v4l2_capability	cap;
    struct v4l2_format      fmt;
//    struct v4l2_frmsizeenum frmsize;
//    struct v4l2_fmtdesc fmtdesc;

    if (SENSOR_TYPE_DTOF == snr_param.sensor_type)
    {
#if defined(RUN_ON_EMBEDDED_LINUX)
        int ret = 0;

        ret = get_subdev_node_4_sensor();
        if (ret < 0)
        {
            DBG_ERROR("Fail to get subdev node for dtof sensor...");
            return 0 - __LINE__;
        }

        if ((fd_4_dtof = open(sd_devnode_4_dtof, O_RDWR)) == -1)
        {
            DBG_ERROR("Fail to open device %s , errno: %s (%d)...", 
                sd_devnode_4_dtof, strerror(errno), errno);
            return 0 - __LINE__;
        }

#endif
    }

    if ((fd = open(video_dev, O_RDWR)) == -1)
    {
        DBG_ERROR("Fail to open device %s , errno: %s (%d)...", 
            video_dev, strerror(errno), errno);
        fd = 0;
        return 0 - __LINE__;
    }

    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1)
    {
        DBG_ERROR("Fail to query device capabilities, errno: %s (%d)...", 
            strerror(errno), errno);
        return 0 - __LINE__;
    }
    else
    {
        DBG_INFO("driver:\t\t%s", cap.driver);
        DBG_INFO("card:\t\t%s", cap.card);
        DBG_INFO("bus_info:\t%s", cap.bus_info);
        DBG_INFO("version:\t0x%x", cap.version);
        DBG_INFO("capabilities:\t0x%x", cap.capabilities);
        DBG_INFO("device_caps:\t0x%x", cap.capabilities);
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) &&
            !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)) {
        DBG_ERROR("this device seems not support video capture, capabilities: 0x%x...",
            cap.capabilities);
        return 0 - __LINE__;
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        DBG_ERROR("this device seems not support streaming, capabilities: 0x%x...",
            cap.capabilities);
        return 0 - __LINE__;
    }

    if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) {
        buf_type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        DBG_INFO("Buffer type:\tV4L2_BUF_TYPE_VIDEO_CAPTURE");
    } else if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
        buf_type                = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        DBG_INFO("Buffer type:\tV4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE");
    }

#if 0
    fmtdesc.index       = 0;
    fmtdesc.type        = buf_type;
    DBG_INFO("Support formats:");
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != -1) {
        DBG_INFO("\t%d\t%s\t\tfourcc:0x%x(%c%c%c%c)",
            fmtdesc.index + 1, 
            fmtdesc.description, 
            fmtdesc.pixelformat,
            fmtdesc.pixelformat & 0xff,
            fmtdesc.pixelformat >> 8 & 0xff,
            fmtdesc.pixelformat >> 16 & 0xff,
            fmtdesc.pixelformat >> 24 & 0xff
            );
        frmsize.pixel_format = fmtdesc.pixelformat;
        frmsize.index       = 0;
        
        while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) >= 0) {
            if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
                DBG_INFO("\t\tDISCRETE resolution[%d]: %d X %d",frmsize.index, frmsize.discrete.width, frmsize.discrete.height);
            }
            else if (frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE) {
                DBG_INFO("\t\tSTEPWISE resolution[%d]: %d X %d",frmsize.index, frmsize.stepwise.max_width, frmsize.stepwise.max_height);
            }
        
            frmsize.index++;
        }
        fmtdesc.index++;
    }
#endif

    // when config swift work mode, need TDC delay min/max need to know which environment, which measurement is used, so I move this the following lines before set sensor fmt;
#if defined(RUN_ON_EMBEDDED_LINUX)
    if (SENSOR_TYPE_DTOF == snr_param.sensor_type)
    {
        struct adaps_dtof_intial_param param;
        p_misc_device = qApp->get_misc_dev_instance();
        if (NULL_POINTER == p_misc_device)
        {
            DBG_ERROR("p_misc_device is NULL");
            return -1;
        }

        script_loaded = false;
        qApp->set_roi_sram_rotate(false);

#if !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
        if (host_comm)
        {
            UINT32 backuped_script_buf_sz;
            u8 *backuped_script_buffer;     // script buffer backup from CMD_HOST_SIDE_START_CAPTURE
            UINT8 backuped_wkmode;
            UINT32 backuped_roisram_data_sz;
            UINT8 *backuped_roisram_data;

            host_comm->get_backuped_external_config_info(&backuped_wkmode, &backuped_script_buffer, &backuped_script_buf_sz, &backuped_roisram_data, &backuped_roisram_data_sz, &roi_sram_rotate);
            DBG_INFO("backuped_script_buf_sz: %d, backuped_roisram_data_sz: %d, roi_sram_rotate: %d...\n", backuped_script_buf_sz, backuped_roisram_data_sz, roi_sram_rotate);

            roi_sram_loaded = (backuped_roisram_data_sz > 0) ? true: false;
            if (backuped_roisram_data_sz <= ALL_ROISRAM_GROUP_SIZE)
            {
                roi_sram_rotate = false;
            }
            qApp->set_roi_sram_rotate(roi_sram_rotate);

            if (backuped_script_buf_sz || backuped_roisram_data_sz)
            {
                int ret = p_misc_device->send_down_external_config(backuped_wkmode, backuped_script_buf_sz, (const uint8_t* ) backuped_script_buffer, backuped_roisram_data_sz, (const uint8_t* ) backuped_roisram_data, roi_sram_rotate);
                if (0 > ret)
                {
                    char err_msg[] = "work_mode and the register config in script buffer is mismatched";
                    host_comm->report_status(CMD_HOST_SIDE_START_CAPTURE, CMD_DEVICE_SIDE_ERROR_MISMATCHED_WORK_MODE, err_msg, strlen(err_msg));
                    return 0 - __LINE__;
                }

                if (backuped_script_buf_sz)
                {
                    script_loaded = true;
                }
            }
        }
#endif

        param.env_type = snr_param.env_type;
        param.measure_type = snr_param.measure_type;
        param.framerate_type = snr_param.framerate_type;
        param.vcselzonecount_type = AdapsVcselZoneCount4;
        param.power_mode = snr_param.power_mode;
        qApp->get_anchorOffset(&param.rowOffset, &param.colOffset);
        qApp->get_spotSearchingRange(&param.rowSearchingRange, &param.colSearchingRange);

        if (0 > p_misc_device->write_dtof_initial_param(&param))
        {
            return 0 - __LINE__;
        }
    }

#if defined(RUN_ON_EMBEDDED_LINUX) && !defined(VIDIOC_S_FMT_INCLUDE_VIDIOC_SUBDEV_S_FMT)
    if (0 != fd_4_dtof)
    {
        int ret = 0;

        ret = set_param_4_sensor_sub_device(snr_param.raw_width, snr_param.raw_height);
        if (0 > ret) {
            DBG_ERROR("Fail to set_param_4_sensor_sub_device, errno: %s (%d)...", 
                   strerror(errno), errno);
            return ret;
        }
    }
#endif
#endif

    DBG_INFO("VIDIOC_S_FMT %d X %d, pixel_format: 0x%x, raw_width:%d, raw_height:%d...\n",
    snr_param.raw_width, snr_param.raw_height, pixel_format, snr_param.raw_width, snr_param.raw_height);
    CLEAR(fmt);
    fmt.type            = buf_type;
    fmt.fmt.pix.pixelformat = pixel_format;
    fmt.fmt.pix.width  = snr_param.raw_width;
    fmt.fmt.pix.height   = snr_param.raw_height;
    //fmt.fmt.pix.field   = V4L2_FIELD_INTERLACED;
    //fmt.fmt.pix.quantization = V4L2_QUANTIZATION_FULL_RANGE;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
        DBG_ERROR("Fail to set format for dev: %s (%d), errno: %s (%d)...", video_dev, fd,
            strerror(errno), errno);
        return 0 - __LINE__;
    }

    if (ioctl(fd, VIDIOC_G_FMT, &fmt) == -1) //重新读取 结构体，以确认完成设置
    {
        DBG_ERROR("Fail to get format , errno: %s (%d)...", 
            strerror(errno), errno);
        return 0 - __LINE__;
    }
    else {
        DBG_INFO("fmt.type:\t%d", fmt.type);
        DBG_INFO("pix.width:\t%d", fmt.fmt.pix.width);
        DBG_INFO("pix.height:\t%d", fmt.fmt.pix.height);
        DBG_INFO("pix.field:\t%d", fmt.fmt.pix.field);
    }

#if 0
    setfps.type         = buf_type; // 设置摄像头的帧率，这里一般设置为 30fps
    setfps.parm.capture.timeperframe.denominator = 30; //fps=30/1=30
    setfps.parm.capture.timeperframe.numerator = 1;

    if (ioctl(fd, VIDIOC_S_PARM, &setfps) == -1) {
        qDebug("Unable to set fps on <%s> line:%d, errno: %s (%d)...", __FUNCTION__, __LINE__, strerror(errno), errno);
        return 0 - errno;
    }

    if (ioctl(fd, VIDIOC_G_PARM, &setfps) == -1) //重新读取结构体，以确认完成设置
    {
        qDebug() << "Unable to get fps";
        return 0 - errno;
    }
    else {
        qDebug() << "fps:\t" << setfps.parm.capture.timeperframe.denominator / setfps.parm.capture.timeperframe.numerator;
    }
#endif

    if (false == alloc_buffers())
    {
        return 0 - __LINE__;
    }

    power_on = true;
    DBG_INFO("init dev %s [OK]", video_dev);

    return 0;
}

int V4L2::Start_streaming(void)
{
    firstFrameTimeUsec = 0;
    rxFrameCnt = 0;
    mipi_rx_fps = 0;
    streamed_timeUs = 0;

    if (-1 == ioctl(fd, VIDIOC_STREAMON, &buf_type))
    {
        DBG_ERROR("Fail to stream_on, errno: %s (%d)...", 
            strerror(errno), errno);
        return 0 - __LINE__;
    }
    stream_on = true;
    DBG_INFO("Start to streaming for dev %s", video_dev);

#if defined(RUN_ON_EMBEDDED_LINUX)
    if (SENSOR_TYPE_DTOF == snr_param.sensor_type)
    {
        struct adaps_dtof_exposure_param *p_exposure_param;

        p_misc_device = qApp->get_misc_dev_instance();
        if (NULL_POINTER == p_misc_device)
        {
            DBG_ERROR("p_misc_device is NULL");
            return -1;
        }

        if (0 > p_misc_device->read_dtof_exposure_param())
        {
            return 0 - __LINE__;
        }

        p_exposure_param = (struct adaps_dtof_exposure_param *) p_misc_device->get_dtof_exposure_param();
        if (NULL_POINTER == p_exposure_param) {
            DBG_ERROR("p_exposure_param is NULL");
            return 0 - __LINE__;
        }

        snr_param.exposureParam.exposure_period= p_exposure_param->exposure_period;
        snr_param.exposureParam.ptm_coarse_exposure_value = p_exposure_param->ptm_coarse_exposure_value;
        snr_param.exposureParam.ptm_fine_exposure_value = p_exposure_param->ptm_fine_exposure_value;
        snr_param.exposureParam.pcm_gray_exposure_value = p_exposure_param->pcm_gray_exposure_value;
    }
#endif

    return 0;
}

int V4L2::Capture_frame()
{
    struct v4l2_buffer  v4l2_buf;
    struct v4l2_plane v4l2_planes[FMT_NUM_PLANES];
    int bytesused;
    struct timeval tv;
    long currTimeUsec;
    status_params1 param1;
#if defined(RUN_ON_EMBEDDED_LINUX)
#if !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
    frame_buffer_param_t param;
#endif
    struct adaps_dtof_runtime_status_param *p_runtime_status_param;
    p_misc_device = qApp->get_misc_dev_instance();
#endif

    CLEAR(v4l2_buf);

    v4l2_buf.type = buf_type;
    v4l2_buf.memory = V4L2_MEMORY_MMAP;

    CLEAR(v4l2_planes);
    if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == buf_type) {
        v4l2_buf.m.planes = v4l2_planes;
        v4l2_buf.length = FMT_NUM_PLANES;
    }

    if (0 == fd || ioctl(fd, VIDIOC_DQBUF, &v4l2_buf) == -1) {
        DBG_ERROR("Fail to dequeue buffer, fd: %d errno: %s (%d)...", fd,
            strerror(errno), errno);
        return 0 - __LINE__;
    }

    if (v4l2_buf.flags & V4L2_BUF_FLAG_ERROR) {
        u64 rxTimeUsec;
        rxTimeUsec = v4l2_buf.timestamp.tv_sec*1000000 + v4l2_buf.timestamp.tv_usec;

        DBG_ERROR("Error (flags:0x%x) in dequeue frame#%d timestampUs: %lld buffer...", 
            v4l2_buf.flags, v4l2_buf.sequence, rxTimeUsec);
        goto error_exit;
    }

    if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == buf_type)
        bytesused = v4l2_buf.m.planes[0].bytesused;
    else
        bytesused = v4l2_buf.bytesused;

    gettimeofday(&tv,NULL_POINTER);
    rxFrameCnt++;

    if (rxFrameCnt < 10 || (0 == rxFrameCnt %50))
    {
        DBG_INFO("Rx %ld frames for dev %s", rxFrameCnt, video_dev);
    }

    if (0 == firstFrameTimeUsec)
    {
        firstFrameTimeUsec = tv.tv_sec*1000000 + tv.tv_usec;

        switch (pixel_format) {
            case V4L2_PIX_FMT_YUYV:
                bits_per_pixel = 12;
                break;

            case V4L2_PIX_FMT_NV12:
                bits_per_pixel = 12;
            break;

            case V4L2_PIX_FMT_SBGGR8:
            default:
                bits_per_pixel = 8;
                break;
        }
        total_bytes_per_line = bytesused/snr_param.raw_height;
        payload_bytes_per_line = (snr_param.raw_width * bits_per_pixel)/8;
        padding_bytes_per_line = total_bytes_per_line - payload_bytes_per_line;
#if defined(RUN_ON_EMBEDDED_LINUX)
        DBG_NOTICE("------script_loaded: %d, workmode: %d, frame raw size: %d X %d, bits_per_pixel: %d, payload_bytes_per_line: %d, total_bytes_per_line: %d, padding_bytes_per_line: %d, frame_buffer_size: %d---\n",
            script_loaded, snr_param.work_mode,
            snr_param.raw_width, snr_param.raw_height,
            bits_per_pixel, payload_bytes_per_line, total_bytes_per_line, padding_bytes_per_line, bytesused);
#else
        DBG_NOTICE("------workmode: %d, frame raw size: %d X %d, bits_per_pixel: %d, payload_bytes_per_line: %d, total_bytes_per_line: %d, padding_bytes_per_line: %d, frame_buffer_size: %d---\n",
            snr_param.work_mode,
            snr_param.raw_width, snr_param.raw_height,
            bits_per_pixel, payload_bytes_per_line, total_bytes_per_line, padding_bytes_per_line, bytesused);
#endif
    }
    else {
        currTimeUsec = tv.tv_sec*1000000 + tv.tv_usec;
        streamed_timeUs = (currTimeUsec - firstFrameTimeUsec);
        mipi_rx_fps = (rxFrameCnt * 1000000) / streamed_timeUs;
    }

    param1.mipi_rx_fps = mipi_rx_fps;
    param1.streamed_time_us = streamed_timeUs;
#if defined(RUN_ON_EMBEDDED_LINUX)
    if (NULL_POINTER == p_misc_device)
    {
        DBG_ERROR("p_misc_device is NULL");
        return -1;
    }

    p_runtime_status_param = (struct adaps_dtof_runtime_status_param *) p_misc_device->get_dtof_runtime_status_param();
    param1.curr_temperature = p_runtime_status_param->inside_temperature_x100;
    param1.curr_exp_vop_abs = p_runtime_status_param->expected_vop_abs_x100;
    param1.curr_exp_pvdd = p_runtime_status_param->expected_pvdd_x100;

    if (Utils::is_env_var_true(ENV_VAR_RAW_FILE_REPLAY_ENABLE))
    {
        if (true == utils->is_replay_data_exist())
        {
            int ret = utils->loadNextFileToBuffer((char *) buffers[v4l2_buf.index].start, bytesused);
            if (ret != bytesused)
            {
                DBG_ERROR("Fail to loadNextFileToBuffer, ret: %d, bytesused: %d!", ret, bytesused);
            }
        }
    }

#if !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
    if (host_comm)
    {
        param.work_mode = snr_param.work_mode;
        param.data_type = FRAME_RAW_DATA;
        param.frm_width = snr_param.raw_width;
        param.frm_height = snr_param.raw_height;
        param.padding_bytes_per_line = padding_bytes_per_line;
        param.env_type = snr_param.env_type;
        param.measure_type = snr_param.measure_type;
        param.framerate_type = snr_param.framerate_type;
        param.power_mode = snr_param.power_mode;
        param.curr_pvdd = p_runtime_status_param->expected_pvdd_x100;
        param.curr_vop_abs = p_runtime_status_param->expected_vop_abs_x100;
        param.curr_inside_temperature = p_runtime_status_param->inside_temperature_x100;
        param.exposure_period = snr_param.exposureParam.exposure_period;
        param.ptm_coarse_exposure_value = snr_param.exposureParam.ptm_coarse_exposure_value;
        param.ptm_fine_exposure_value = snr_param.exposureParam.ptm_fine_exposure_value;
        param.pcm_gray_exposure_value = snr_param.exposureParam.pcm_gray_exposure_value;
        param.frame_sequence = v4l2_buf.sequence;
        param.frame_timestamp_us = timestamp_convert_from_timeval_to_us(v4l2_buf.timestamp);
        param.mipi_rx_fps = mipi_rx_fps;

        host_comm->report_frame_raw_data(buffers[v4l2_buf.index].start, bytesused, &param);
    }
#endif
#endif

    if (true != Utils::is_env_var_true(ENV_VAR_SKIP_FRAME_PROCESS))
    {
        emit update_info(param1);
#if defined(RUN_ON_EMBEDDED_LINUX) && !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
        emit rx_new_frame(v4l2_buf.sequence, buffers[v4l2_buf.index].start, bytesused, v4l2_buf.timestamp, frm_type, total_bytes_per_line, param);
#else
        emit rx_new_frame(v4l2_buf.sequence, buffers[v4l2_buf.index].start, bytesused, v4l2_buf.timestamp, frm_type, total_bytes_per_line);
#endif
    }
    else {
        if (1 == v4l2_buf.sequence % FRAME_INTERVAL_4_PROGRESS_REPORT)
        {
            DBG_NOTICE("%s() mipi_rx_fps = %d fps, rxFrameCnt: %ld\n",  __FUNCTION__, mipi_rx_fps, rxFrameCnt);
        }
    }

error_exit:
    if (0 == fd || -1 == ioctl(fd, VIDIOC_QBUF, &v4l2_buf)) {
        DBG_ERROR("Fail to queue buffer, errno: %s (%d)...", 
            strerror(errno), errno);
        return 0 - __LINE__;
    }

    return 0;
}

void V4L2::Stop_streaming(void)
{
    int streamed_second = streamed_timeUs/1000000;

    stream_on = false;

    if (rxFrameCnt <= 0) // not started yet.
        return;

    if (0 != fd)
    {
        DBG_NOTICE("------streaming statistics------streamed: %02d:%02d:%02d, rxFrameCnt: %ld, mipi_rx_fps: %d, frame raw size: %d X %d---\n",
            streamed_second/3600,streamed_second/60,streamed_second%60,
            rxFrameCnt, mipi_rx_fps, 
            snr_param.raw_width, snr_param.raw_height);
        
        if (-1 == ioctl(fd, VIDIOC_STREAMOFF, &buf_type)) {
            DBG_ERROR("Fail to stream off, errno: %s (%d)...", 
                strerror(errno), errno);
        }
    }

    return;
}

void V4L2::V4l2_close(void)
{
    if (SENSOR_TYPE_DTOF == snr_param.sensor_type)
    {
#if defined(RUN_ON_EMBEDDED_LINUX)
        if (0 != fd_4_dtof){
            if (-1 == close(fd_4_dtof)) {
                DBG_ERROR("Fail to close device %d (%s), errno: %s (%d)...", fd_4_dtof, sd_devnode_4_dtof,
                    strerror(errno), errno);
                return;
            }
            fd_4_dtof = 0;
        }
#endif
    }

    free_buffers();
    if (0 != fd)
    {
        if (-1 == close(fd)) {
            DBG_ERROR("Fail to close device, errno: %s (%d)...", 
                strerror(errno), errno);
            return;
        }
        fd = 0;
    }
    power_on = false;

    return;
}

void V4L2::Get_frame_size_4_curr_wkmode(int *in_width, int *in_height, int *out_width, int *out_height)
{
    *in_width = snr_param.raw_width;
    *in_height = snr_param.raw_height;
    *out_width = snr_param.out_frm_width;
    *out_height = snr_param.out_frm_height;
}

