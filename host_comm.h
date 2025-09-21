#ifndef HOST_COMMUNICATION_H
#define HOST_COMMUNICATION_H

#if defined(RUN_ON_EMBEDDED_LINUX) && !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)

#include <QObject>
#include <QString>
#include <QMetaType>

#include "adaps_types.h"
#include "adaps_sender.h"
#include "host_device_comm_types.h"
#include "misc_device.h"

Q_DECLARE_METATYPE(capture_req_param_t*);

class Host_Communication : public QObject
{
Q_OBJECT

public:
    // 删除拷贝构造和赋值操作（防止复制单例）
    Host_Communication(const Host_Communication&) = delete;
    Host_Communication& operator=(const Host_Communication&) = delete;
    ~Host_Communication();

    int report_frame_raw_data(void* pFrameData, uint32_t frameData_size, frame_buffer_param_t *pFrmBufParam);
    int report_frame_depth16_data(void* pFrameData, uint32_t frameData_size, frame_buffer_param_t *pFrmBufParam);
    int report_frame_pointcloud_data(void* pFrameData, uint32_t frameData_size, frame_buffer_param_t *pFrmBufParam);
    int report_req_histogram_data(void* pHistDataPtr, uint32_t spotHistDataSize, frame_buffer_param_t *pFrmBufParam);
    void get_backuped_external_config_script(UINT8 *workmode, UINT8 ** script_buffer, uint32_t *script_buffer_size);
    int report_status(UINT16 responsed_cmd, UINT16 status_code, char *msg, int msg_length);
    UINT8 get_req_out_data_type();
    UINT8 get_req_walkerror_enable();
    BOOLEAN get_req_compose_subframe();
    BOOLEAN get_req_expand_pixel();
    BOOLEAN get_req_mirror_x();
    BOOLEAN get_req_mirror_y();
    void* get_loaded_ref_distance_data();
    void* get_loaded_lens_intrinsic_data();
    uint16_t get_req_histogram_x();
    uint16_t get_req_histogram_y();

    // 获取单例实例
    static Host_Communication* getInstance();

signals:
    void start_capture();
    void stop_capture();
    void set_capture_options(capture_req_param_t* param);

private slots:
    //void on_eeprom_readout(void* pEEPROM_buffer, int EEPROM_size);

private:
    static Host_Communication* instance; // 静态成员变量声明
    bool connected;
    UINT32 backuped_script_buffer_size;
    UINT8 *backuped_script_buffer = NULL_POINTER;     // script buffer backuped from CMD_HOST_SIDE_START_CAPTURE
    UINT8 backuped_wkmode;
    capture_req_param_t backuped_capture_req_param;
    UINT32 loaded_walkerror_data_size;
    UINT8 *loaded_walkerror_data = NULL_POINTER;     // roisram_data backuped from CMD_HOST_SIDE_SET_SPOT_WALKERROR_DATA
    UINT32 loaded_spotoffset_data_size;
    UINT8 *loaded_spotoffset_data = NULL_POINTER;
    float *loaded_ref_distance_data = NULL_POINTER;     // roisram_data backuped from CMD_HOST_SIDE_SET_REF_DISTANCE_DATA
    float *loaded_lens_intrinsic_data = NULL_POINTER;     // roisram_data backuped from CMD_HOST_SIDE_SET_LENS_INTRINSIC_DATA
    UINT32 eeprom_capacity;       // unit is byte
    Misc_Device *p_misc_device;

    unsigned long txRawdataFrameCnt;
    unsigned long firstRawdataFrameTimeUsec;
    UINT32 txRawdataFrame_fps = 0;
    unsigned long txDepth16FrameCnt;
    unsigned long firstDepth16FrameTimeUsec;
    UINT32 txDepth16Frame_fps = 0;
    unsigned long txPointCloudFrameCnt;
    UINT32 txPointCloudFrame_fps = 0;
    unsigned long firstPointCloudFrameTimeUsec;
    uint16_t req_histogram_x = 0;
    uint16_t req_histogram_y = 0;

    Host_Communication(); // 私有构造函数（防止外部实例化）

    void reset_data();
    static int adaps_sender_callback(SenderEventId_t id, void* arg_ptr, uint32_t arg_u32, ...);

    void adaps_start_capture(CommandData_t* pCmdData, uint32_t rxDataLen);
    void adaps_set_colormap_range(CommandData_t* pCmdData, uint32_t rxDataLen);
    void read_device_register(UINT16 cmdType, CommandData_t* pCmdData, uint32_t rxDataLen);
    void write_device_register(UINT16 cmdType, CommandData_t* pCmdData, uint32_t rxDataLen);
    int report_module_static_data();
    void adaps_load_roi_sram(CommandData_t* pCmdData, uint32_t rxDataLen);
    void adaps_load_walkerror_data(CommandData_t* pCmdData, uint32_t rxDataLen);
    void adaps_load_spotoffset_data(CommandData_t* pCmdData, uint32_t rxDataLen);
    void adaps_set_walkerror_enable(CommandData_t* pCmdData, uint32_t rxDataLen);
    void adaps_update_eeprom_data(CommandData_t* pCmdData, uint32_t rxDataLen);
    void adaps_request_device_reboot(CommandData_t* pCmdData, uint32_t rxDataLen);
    void adaps_set_rtc_time(CommandData_t* pCmdData, uint32_t rxDataLen);
    void adaps_load_ref_distance_data(CommandData_t* pCmdData, uint32_t rxDataLen);
    void adaps_load_lens_intrinsic_data(CommandData_t* pCmdData, uint32_t rxDataLen);
    void adaps_set_req_histogram_position(CommandData_t* pCmdData, uint32_t rxDataLen);

    void adaps_event_process(void* pRXData, uint32_t rxDataLen);
    void adaps_sender_disconnected(void);
    void adaps_sender_connected(void);
    int adaps_sender_init(void);
    int dump_buffer_data(void* dump_buf, const char *buffer_name, int callline);
    int dump_capture_req_param(capture_req_param_t* pCaptureReqParam);
    int dump_frame_param(frame_buffer_param_t *pDataBufferParam);
    int dump_module_static_data(module_static_data_t *pStaticDataParam);
    void backup_roi_sram_data(roisram_data_param_t* pRoiSramParam);
    bool save_data_2_bin_file(const char *prefix, void *buf, int len);

};

#endif // defined(RUN_ON_EMBEDDED_LINUX) && !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)

#endif // HOST_COMMUNICATION_H

