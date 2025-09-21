#ifndef GLOBALAPPLICATION_H
#define GLOBALAPPLICATION_H

#include <QObject>
#if !defined(CONSOLE_APP_WITHOUT_GUI)
	#include <QApplication>
#else
	#include <QCoreApplication>
#endif
#include <common.h>
#include"v4l2.h"
#include "misc_device.h"

class GlobalApplication;

#if defined(qApp)
#undef qApp
#endif
#define qApp (static_cast<GlobalApplication *>(QCoreApplication::instance()))

#if !defined(CONSOLE_APP_WITHOUT_GUI)
class GlobalApplication : public QApplication
#else
class GlobalApplication : public QCoreApplication
#endif
{
    Q_OBJECT
public:
    GlobalApplication(int argc, char *argv[]);
    sensor_workmode get_wk_mode();
    sensortype get_sensor_type();
    int get_save_cnt();
    int get_timer_test_times();
    bool get_qt_ui_test();
    int set_wk_mode(int wk_mode);
    int set_sensor_type(int sensortype);
    V4L2* get_v4l2_instance();
    int register_v4l2_instance(V4L2* new_instance);

#if defined(RUN_ON_EMBEDDED_LINUX)
    Misc_Device* get_misc_dev_instance();
    int register_misc_dev_instance(Misc_Device* new_instance);
    bool is_capture_req_from_host();
    int set_capture_req_from_host(bool val);
    AdapsPowerMode get_power_mode();
    int set_power_mode(int power_mode);
    AdapsFramerateType get_framerate_type();
    int set_framerate_type(int framerate_type);
    AdapsEnvironmentType get_environment_type();
    int set_environment_type(int environment_type);
    AdapsMeasurementType get_measurement_type();
    int set_measurement_type(int mtype);

    int get_GrayScaleMinMappedRange();
    int set_GrayScaleMinMappedRange(int value);

    int get_GrayScaleMaxMappedRange();
    int set_GrayScaleMaxMappedRange(int value);

    float get_RealDistanceMinMappedRange();
    int set_RealDistanceMinMappedRange(float value);

    float get_RealDistanceMaxMappedRange();
    int set_RealDistanceMaxMappedRange(float value);

    bool is_roi_sram_rolling();
    int set_roi_sram_rolling(bool val);

    UINT8* get_mmap_address_4_loaded_roisram();
    int set_mmap_address_4_loaded_roisram(UINT8* addr);
    UINT32 get_size_4_loaded_roisram();
    int set_size_4_loaded_roisram(UINT32 bytes);

    UINT8* get_loaded_walkerror_data();
    int set_loaded_walkerror_data(UINT8* value);
    UINT32 get_loaded_walkerror_data_size();
    int set_loaded_walkerror_data_size(UINT32 value);

    float* get_loaded_spotoffset_data();
    int set_loaded_spotoffset_data(UINT8* value);
    UINT32 get_loaded_spotoffset_data_size();
    int set_loaded_spotoffset_data_size(UINT32 value);

    bool is_walkerror_enabled();
    int set_walkerror_enable(bool val);

    moduletype get_module_type();
    int set_module_type(int moduletype);

    int get_anchorOffset(UINT8 *rowOffset, UINT8 *colOffset);
    int set_anchorOffset(UINT8 rowOffset, UINT8 colOffset);

    int get_spotSearchingRange(UINT8 *rowSearchingRange, UINT8 *colSearchingRange);
    int set_spotSearchingRange(UINT8 rowSearchingRange, UINT8 colSearchingRange);
    int get_usrCfgExposureValues(UINT8 *coarseExposure, UINT8 *fineExposure, UINT8 *grayExposure, UINT8 *laserExposurePeriod);
    int set_usrCfgExposureValues(UINT8 coarseExposure, UINT8 fineExposure, UINT8 grayExposure, UINT8 laserExposurePeriod);
#endif

private:

    moduletype      selected_module_type;
    sensortype      selected_sensor_type;
    sensor_workmode selected_wk_mode;
    int             save_frame_cnt;
    int             timer_test_times;
    bool            qt_ui_test;
    V4L2            *v4l2_instance;

#if defined(RUN_ON_EMBEDDED_LINUX)
    AdapsMeasurementType selected_m_type;
    AdapsEnvironmentType selected_e_type;
    AdapsFramerateType selected_framerate_type;
    AdapsPowerMode selected_power_mode;
    bool            capture_req_from_host;
    bool            roi_sram_rolling;
    bool            walkerror_enable;
    int GrayScaleMinMappedRange;
    int GrayScaleMaxMappedRange;
    float RealDistanceMinMappedRange;
    float RealDistanceMaxMappedRange;
    UINT8 *loaded_walkerror_data;
    UINT32 loaded_walkerror_data_size;
    UINT8 *loaded_spotoffset_data;
    UINT32 loaded_spotoffset_data_size;
    UINT8 *mmap_address_4_loaded_roisram;
    UINT32 size_4_loaded_roisram;
    Misc_Device     *misc_dev_instance;
    UINT8 anchor_rowOffset;
    UINT8 anchor_colOffset;
    UINT8 rowSearchingRange;
    UINT8 colSearchingRange;
    UINT8 usrCfgGrayExposure;
    UINT8 usrCfgCoarseExposure;
    UINT8 usrCfgFineExposure;
    UINT8 usrCfgLaserExposurePeriod;

    AdapsMeasurementType string_2_measurementtype(QString& str);
    AdapsEnvironmentType string_2_environmenttype(QString& str);
    AdapsPowerMode string_2_powermode(QString& str);
#endif
    sensortype string_2_sensortype(QString& str);
    sensor_workmode string_2_workmode(QString& str);
};

#endif // GLOBALAPPLICATION_H

