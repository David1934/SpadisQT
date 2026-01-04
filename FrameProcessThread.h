#ifndef FrameProcessThread_H
#define FrameProcessThread_H

#include <QThread>

#if !defined(CONSOLE_APP_WITHOUT_GUI)
#include <QImage>
#endif

#include <QDebug>
#include"v4l2.h"
#if defined(RUN_ON_EMBEDDED_LINUX)
#include "adaps_dtof.h"
#endif
#include"utils.h"

class FrameProcessThread : public QThread
{
    Q_OBJECT
public:
    FrameProcessThread();
    ~FrameProcessThread();

    void stop(int stop_request_code);
    int init(int index);
    bool isSleeping();
    #if !defined(CONSOLE_APP_WITHOUT_GUI)
        void setWatchSpot(QSize img_widget_size, QPoint point);
    #endif

protected:
    void run();

private:
    volatile int majorindex;
    volatile bool stopped;
    volatile bool sleeping;
    volatile bool skip_frame_decode;
    volatile uint32_t dump_spot_statistics_times;
    volatile uint32_t dump_ptm_frame_headinfo_times;
    struct sensor_params sns_param;
    char *expected_md5_string;

    #if defined(RUN_ON_EMBEDDED_LINUX)
        ADAPS_DTOF *adaps_dtof;
        #if ALGO_LIB_VERSION_CODE >= VERSION_HEX_VALUE(3, 5, 6) && defined(ENABLE_POINTCLOUD_OUTPUT)
            pc_pkt_t* out_pcloud_buffer;
            u32 out_pcloud_buffer_size;
        #endif
        u16 *depth_buffer;
        u32 depth_buffer_size;
        unsigned char *confidence_map_buffer;
    #endif
    Utils *utils;
    V4L2 *v4l2;
    unsigned char *rgb_buffer;
    uint32_t outputed_frame_cnt;
    uint32_t dumped_frame_cnt;
    int stop_req_code;
    bool save_frame(unsigned int frm_sequence, void *frm_buf, int buf_size, int frm_w, int frm_h, struct timeval frm_timestamp, enum frame_data_type);

private slots:

#if defined(RUN_ON_EMBEDDED_LINUX) && !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
    bool new_frame_handle(unsigned int frm_sequence, void *frm_buf, int buf_len, struct timeval frm_timestamp, enum frame_data_type, int total_bytes_per_line, frame_buffer_param_t frmBufParam);
#else
    bool new_frame_handle(unsigned int frm_sequence, void *frm_buf, int buf_len, struct timeval frm_timestamp, enum frame_data_type, int total_bytes_per_line);
#endif
    bool info_update(status_params1 param1);
    void onThreadLoopExit();

signals:

    #if !defined(CONSOLE_APP_WITHOUT_GUI)
        void newFrameReady4Display(unsigned int frm_sequence, QImage image, QImage img4confidence);
    #endif
    bool update_runtime_display(status_params2 param2);
    void threadLoopExit();
    void threadEnd(int exit_request_code);
};

#endif // FrameProcessThread_H
