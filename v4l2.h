#ifndef V4L2_H
#define V4L2_H

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
#include <semaphore.h>
#include <QDebug>
#include <QDateTime>

#include "common.h"
#include "host_comm.h"
#include "misc_device.h"
#include"utils.h"

#define BUFFER_COUNT_4_DTOF_SENSOR              6
#define OUTPUT_WIDTH_4_DTOF_SENSOR              210
#define OUTPUT_HEIGHT_4_DTOF_SENSOR             160

#define MIPI_RAW_HEIGHT_4_DTOF_SENSOR           32

#define BUFFER_COUNT_4_RGB_SENSOR               5
#define RGB_IMAGE_CHANEL                        3

#define DEV_NODE_LEN                            32
#define FMT_NUM_PLANES                          1


struct buffer_s
{
    void *start;
    unsigned int length;
};

struct sensor_data
{
    const char *sensor_subdev;
    const char *media_devnode;
    const char *video_devnode;
    int raw_w;
    int raw_h;
    int out_frm_width;
    int out_frm_height;
    int pixfmt;
    int frm_buf_cnt; 
    enum sensortype stype;
    enum frame_data_type ftype;
};

#if defined(RUN_ON_EMBEDDED_LINUX) && !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
Q_DECLARE_METATYPE(frame_buffer_param_t);
#endif

class V4L2 : public QObject
{
Q_OBJECT

public:
    V4L2(struct sensor_params params);
    ~V4L2();

    int V4l2_initilize(void);
    int Start_streaming(void);
    int Capture_frame();
    void Stop_streaming(void);
    void V4l2_close(void);
    void Get_frame_size_4_curr_wkmode(int *in_width, int *in_height, int *out_width, int *out_height);
    bool get_power_on_state();
    bool get_stream_on_state();
    int get_videodev_fd();


private:
    enum frame_data_type frm_type;
    int fd;
    struct buffer_s *buffers;
    unsigned long firstFrameTimeUsec;
    unsigned long rxFrameCnt;
    int mipi_rx_fps;
    unsigned long streamed_timeUs;
    bool            power_on;
    bool            stream_on;

    struct v4l2_requestbuffers	req_bufs;
    enum	v4l2_buf_type	buf_type;
    unsigned int	pixel_format;
    int bits_per_pixel;
    int padding_bytes_per_line;
    int total_bytes_per_line;
    int payload_bytes_per_line;

    char        sensor_sd_name[DEV_NODE_LEN];
    char        media_dev[DEV_NODE_LEN];
    char        video_dev[DEV_NODE_LEN];
    struct sensor_params snr_param;
    int         frame_buffer_count;
    struct sensor_data *sensordata;

#if defined(RUN_ON_EMBEDDED_LINUX)
    bool            script_loaded;
    bool            roi_sram_loaded;
    bool            roi_sram_rolling;
    char        sd_devnode_4_dtof[DEV_NODE_LEN];
    int         fd_4_dtof;
#if !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
    Host_Communication *host_comm;
#endif
    Misc_Device *p_misc_device;
    Utils *utils;
#endif

    int init();
    bool alloc_buffers(void);
    void free_buffers(void);
    int get_devnode_from_sysfs(struct media_entity_desc *entity_desc, char *p_devname);
    int get_subdev_node_4_sensor();
#if defined(RUN_ON_EMBEDDED_LINUX)
    int set_param_4_sensor_sub_device(int raw_w_4_curr_wkmode, int raw_h_4_curr_wkmode);
#endif
    UINT64 timestamp_convert_from_timeval_to_us(struct timeval timestamp);

signals:

#if defined(RUN_ON_EMBEDDED_LINUX) && !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
    void rx_new_frame(unsigned int frm_sequence, void *frm_buf, int frm_len, struct timeval frm_timestamp, enum frame_data_type ftype, int total_bytes_per_line, frame_buffer_param_t frmBufParam);
#else
    void rx_new_frame(unsigned int frm_sequence, void *frm_buf, int frm_len, struct timeval frm_timestamp, enum frame_data_type ftype, int total_bytes_per_line);
#endif
    void update_info(status_params1 param1);
};

#endif // V4L2_H
