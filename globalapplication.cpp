#include <globalapplication.h>
#include <QDebug>
#include <string>
#include <QCommandLineParser>

#if !defined(CONSOLE_APP_WITHOUT_GUI)
GlobalApplication::GlobalApplication(int argc, char *argv[]):QApplication(argc, argv)
#else
GlobalApplication::GlobalApplication(int argc, char *argv[]):QCoreApplication(argc, argv)
#endif
{
    bool ok;
    QString option1Value;
    QString option2Value;
    int option3Value;
    int option4Value;

    QCoreApplication::setApplicationName(APP_NAME);
    QCoreApplication::setApplicationVersion(APP_VERSION);

    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption sensor_wk_mode_opt(QStringList() << "m" << "mode", "Work mode for the sensor (PCM PHR FHR NV12 YUYV)", "mode");
    QCommandLineOption sensor_type_opt(QStringList() << "t" << "type", "Type of the sensor (RGB DTOF)", "type");
    QCommandLineOption save_frame_cnt_opt(QStringList() << "s" << "save", "Number of frames to save (>=0)", "count");
    QCommandLineOption timer_test_times_opt(QStringList() << "times", "times to be tested by timer (>=0)", "count");
    QCommandLineOption qt_ui_test_opt(QStringList() << "qttest", "a simple UI display test with QT");
#if defined(RUN_ON_EMBEDDED_LINUX)
    QCommandLineOption environment_type_opt(QStringList() << "e" << "etype", "Type of the environment (Indoor Outdoor)", "etype");
    QCommandLineOption measurement_type_opt(QStringList() << "M" << "mtype", "Type of the measurement (Normal Short Full)", "mtype");
    QCommandLineOption power_mode_opt(QStringList() << "p" << "pmode", "Power mode for the sensor (Div1 Div3)", "pmode");
    //QCommandLineOption no_host_comm_opt(QStringList() << "nohostcomm", "local run mode without host communication");
    //QCommandLineOption output_data_type_opt(QStringList() << "o" << "otype", "Type of the output data type (Raw Grayscale Depth16 Depth16XY PointCloud) to host", "otype");
#endif

    parser.addOption(sensor_wk_mode_opt);
    parser.addOption(sensor_type_opt);
    parser.addOption(save_frame_cnt_opt);
    parser.addOption(timer_test_times_opt);
    parser.addOption(qt_ui_test_opt);
#if defined(RUN_ON_EMBEDDED_LINUX)
    parser.addOption(environment_type_opt);
    parser.addOption(measurement_type_opt);
    parser.addOption(power_mode_opt);
    //parser.addOption(no_host_comm_opt);
    //parser.addOption(output_data_type_opt);
#endif

    //parser.process(qApp->arguments());
    parser.process(*this);
    qt_ui_test = false;
    timer_test_times = 0;
    selected_sensor_type = DEFAULT_SENSOR_TYPE;
    selected_wk_mode = DEFAULT_WORK_MODE;
    save_frame_cnt = DEFAULT_SAVE_FRAME_CNT;
    timer_test_times = DEFAULT_TIMER_TEST_TIMES;
    qt_ui_test = false;
    v4l2_instance = nullptr;

#if defined(RUN_ON_EMBEDDED_LINUX)
    misc_dev_instance = nullptr;
    GrayScaleMinMappedRange = 50;
    GrayScaleMaxMappedRange = 2000;
    RealDistanceMinMappedRange = 0.0f;
    RealDistanceMaxMappedRange = 4.0f;
    selected_e_type = DEFAULT_ENVIRONMENT_TYPE;
    selected_m_type = DEFAULT_MEASUREMENT_TYPE;
    selected_framerate_type = DEFAULT_DTOF_FRAMERATE;
    selected_power_mode = AdapsPowerModeNormal;
    capture_req_from_host = false;
    roi_sram_rolling = false;
    loaded_walkerror_data = nullptr;
    loaded_walkerror_data_size = 0;
    loaded_spotoffset_data = nullptr;
    loaded_spotoffset_data_size = 0;
    walkerror_enable = false;
    selected_module_type = MODULE_TYPE_SPOT;
    anchor_colOffset = 0;
    anchor_rowOffset = 0;
    rowSearchingRange = 2;
    colSearchingRange = 2;
    usrCfgGrayExposure = 0;
    usrCfgCoarseExposure = 0;
    usrCfgFineExposure = 0;
#endif

    //DBG_INFO( "---------------");
    if (parser.isSet(sensor_wk_mode_opt)) {
        //DBG_INFO( "---------------");
        option1Value = parser.value(sensor_wk_mode_opt);
        selected_wk_mode = string_2_workmode(option1Value);
    }

    if (parser.isSet(sensor_type_opt)) {
        option2Value = parser.value(sensor_type_opt);
        selected_sensor_type = string_2_sensortype(option2Value);
    }

#if defined(RUN_ON_EMBEDDED_LINUX)
    if (parser.isSet(environment_type_opt)) {
        option2Value = parser.value(environment_type_opt);
        selected_e_type = string_2_environmenttype(option2Value);
    }

    if (parser.isSet(measurement_type_opt)) {
        option2Value = parser.value(measurement_type_opt);
        selected_m_type = string_2_measurementtype(option2Value);
    }

    if (parser.isSet(power_mode_opt)) {
        option2Value = parser.value(power_mode_opt);
        selected_power_mode = string_2_powermode(option2Value);
    }
#endif

    if (parser.isSet(save_frame_cnt_opt)) {
        option3Value = parser.value(save_frame_cnt_opt).toInt(&ok);
        if(!ok) {
            DBG_ERROR("Failed to convert save frame count to int. Using default value of 0.");
            option3Value = DEFAULT_SAVE_FRAME_CNT;
        }
        save_frame_cnt = option3Value;
    }

    if (parser.isSet(timer_test_times_opt)) {
        option4Value = parser.value(timer_test_times_opt).toInt(&ok);
        if(!ok) {
            DBG_ERROR("Failed to convert timer test count to int. Using default value of 1.");
            option4Value = DEFAULT_TIMER_TEST_TIMES;
        }
        timer_test_times = option4Value;
    }

    if (parser.isSet(qt_ui_test_opt)) {
        qt_ui_test = true;
    }

    DBG_INFO( "-----selected_wk_mode:%d selected_sensor_type:%d, save_frame_cnt:%d----------", selected_wk_mode, selected_sensor_type,save_frame_cnt);
}

sensor_workmode GlobalApplication::get_wk_mode()
{
    return selected_wk_mode;
}

int GlobalApplication::set_wk_mode(int wk_mode)
{
    int ret = 0;

    if (SENSOR_TYPE_DTOF == selected_sensor_type)
    {
        if (wk_mode >= WK_DTOF_PHR && wk_mode <= WK_DTOF_FHR)
        {
            selected_wk_mode = (sensor_workmode) wk_mode;
        }
        else {
            ret = -1;
        }
    }
    else if (SENSOR_TYPE_RGB == selected_sensor_type)
    {
        if (wk_mode >= WK_RGB_NV12 && wk_mode <= WK_RGB_YUYV)
        {
            selected_wk_mode = (sensor_workmode) wk_mode;
        }
        else {
            ret = -1;
        }
    }
    else {
        ret = -1;
    }


    return ret;
}

V4L2* GlobalApplication::get_v4l2_instance()
{
    return v4l2_instance;
}

int GlobalApplication::register_v4l2_instance(V4L2* new_instance)
{
    int ret = 0;

    v4l2_instance = new_instance;

    return ret;
}

#if defined(RUN_ON_EMBEDDED_LINUX)
Misc_Device* GlobalApplication::get_misc_dev_instance()
{
    return misc_dev_instance;
}

int GlobalApplication::register_misc_dev_instance(Misc_Device* new_instance)
{
    int ret = 0;

    misc_dev_instance = new_instance;

    return ret;
}

bool GlobalApplication::is_capture_req_from_host()
{
    return capture_req_from_host;
}

int GlobalApplication::set_capture_req_from_host(bool val)
{
    int ret = 0;

    capture_req_from_host = val;

    return ret;
}

bool GlobalApplication::is_roi_sram_rolling()
{
    return roi_sram_rolling;
}

int GlobalApplication::set_roi_sram_rolling(bool val)
{
    int ret = 0;

    roi_sram_rolling = val;

    return ret;
}

bool GlobalApplication::is_walkerror_enabled()
{
    return walkerror_enable;
}

int GlobalApplication::set_walkerror_enable(bool val)
{
    int ret = 0;

    walkerror_enable = val;

    return ret;
}

AdapsFramerateType GlobalApplication::get_framerate_type()
{
    return selected_framerate_type;
}

int GlobalApplication::set_framerate_type(int framerate_type)
{
    int ret = 0;

    if (framerate_type >= AdapsFramerateType15FPS && framerate_type <= AdapsFramerateType60FPS)
    {
        selected_framerate_type = (AdapsFramerateType) framerate_type;
    }
    else {
        ret = -1;
    }

    return ret;
}

AdapsPowerMode GlobalApplication::get_power_mode()
{
    return selected_power_mode;
}

int GlobalApplication::set_power_mode(int power_mode)
{
    int ret = 0;

    if (power_mode >= AdapsPowerModeNormal && power_mode <= AdapsPowerModeDiv3)
    {
        selected_power_mode = (AdapsPowerMode) power_mode;
    }
    else {
        ret = -1;
    }

    return ret;
}

AdapsEnvironmentType GlobalApplication::get_environment_type()
{
    return selected_e_type;
}

int GlobalApplication::set_environment_type(int environment_type)
{
    int ret = 0;

    if (AdapsEnvTypeIndoor == environment_type || AdapsEnvTypeOutdoor == environment_type)
    {
        selected_e_type = (AdapsEnvironmentType) environment_type;
    }
    else {
        ret = -1;
    }

    return ret;
}

AdapsMeasurementType GlobalApplication::get_measurement_type()
{
    return selected_m_type;
}

int GlobalApplication::set_measurement_type(int mtype)
{
    int ret = 0;

    if (mtype >= AdapsMeasurementTypeNormal && mtype <= AdapsMeasurementTypeFull)
    {
        selected_m_type = (AdapsMeasurementType) mtype;
    }
    else {
        ret = -1;
    }

    return ret;
}

int GlobalApplication::get_GrayScaleMinMappedRange()
{
    return GrayScaleMinMappedRange;
}

int GlobalApplication::set_GrayScaleMinMappedRange(int value)
{
    int ret = 0;

    GrayScaleMinMappedRange = value;

    return ret;
}

int GlobalApplication::get_GrayScaleMaxMappedRange()
{
    return GrayScaleMaxMappedRange;
}

int GlobalApplication::set_GrayScaleMaxMappedRange(int value)
{
    int ret = 0;

    GrayScaleMaxMappedRange = value;

    return ret;
}

float GlobalApplication::get_RealDistanceMinMappedRange()
{
    return RealDistanceMinMappedRange;
}

int GlobalApplication::set_RealDistanceMinMappedRange(float value)
{
    int ret = 0;

    RealDistanceMinMappedRange = value;

    return ret;
}

float GlobalApplication::get_RealDistanceMaxMappedRange()
{
    return RealDistanceMaxMappedRange;
}

int GlobalApplication::set_RealDistanceMaxMappedRange(float value)
{
    int ret = 0;

    RealDistanceMaxMappedRange = value;

    return ret;
}

UINT8* GlobalApplication::get_loaded_walkerror_data()
{
    return loaded_walkerror_data;
}

int GlobalApplication::set_loaded_walkerror_data(UINT8* value)
{
    int ret = 0;

    loaded_walkerror_data = value;

    return ret;
}

UINT32 GlobalApplication::get_loaded_walkerror_data_size()
{
    return loaded_walkerror_data_size;
}

int GlobalApplication::set_loaded_walkerror_data_size(UINT32 value)
{
    int ret = 0;

    loaded_walkerror_data_size = value;

    return ret;
}

UINT8* GlobalApplication::get_mmap_address_4_loaded_roisram()
{
    return mmap_address_4_loaded_roisram;
}

int GlobalApplication::set_mmap_address_4_loaded_roisram(UINT8* addr)
{
    int ret = 0;

    mmap_address_4_loaded_roisram = addr;

    return ret;
}

UINT32 GlobalApplication::get_size_4_loaded_roisram()
{
    return size_4_loaded_roisram;
}

int GlobalApplication::set_size_4_loaded_roisram(UINT32 bytes)
{
    int ret = 0;

    size_4_loaded_roisram = bytes;

    return ret;
}

float* GlobalApplication::get_loaded_spotoffset_data()
{
    return (float *) loaded_spotoffset_data;
}

int GlobalApplication::set_loaded_spotoffset_data(UINT8* value)
{
    int ret = 0;

    loaded_spotoffset_data = value;

    return ret;
}

UINT32 GlobalApplication::get_loaded_spotoffset_data_size()
{
    return loaded_spotoffset_data_size;
}

int GlobalApplication::set_loaded_spotoffset_data_size(UINT32 value)
{
    int ret = 0;

    loaded_spotoffset_data_size = value;

    return ret;
}

moduletype GlobalApplication::get_module_type()
{
    return selected_module_type;
}

int GlobalApplication::set_module_type(int module_type)
{
    int ret = 0;

    selected_module_type = (moduletype) module_type;

    return ret;
}

int GlobalApplication::get_anchorOffset(UINT8 *rowOffset, UINT8 *colOffset)
{
    int ret = 0;

    *colOffset = anchor_colOffset;
    *rowOffset = anchor_rowOffset;

    return ret;
}

int GlobalApplication::set_anchorOffset(UINT8 rowOffset, UINT8 colOffset)
{
    int ret = 0;

    anchor_colOffset = colOffset;
    anchor_rowOffset = rowOffset;

    return ret;
}

int GlobalApplication::get_spotSearchingRange(UINT8 *rowRange, UINT8 *colRange)
{
    int ret = 0;

    *rowRange = rowSearchingRange;
    *colRange = colSearchingRange;

    return ret;
}

int GlobalApplication::set_spotSearchingRange(UINT8 rowRange, UINT8 colRange)
{
    int ret = 0;

    rowSearchingRange = rowRange;
    colSearchingRange = colRange;

    return ret;
}

int GlobalApplication::get_usrCfgExposureValues(UINT8 *coarseExposure, UINT8 *fineExposure, UINT8 *grayExposure, UINT8 *laserExposurePeriod)
{
    int ret = 0;

    *coarseExposure = usrCfgCoarseExposure;
    *fineExposure = usrCfgFineExposure;
    *grayExposure = usrCfgGrayExposure;
    *laserExposurePeriod = usrCfgLaserExposurePeriod;

    return ret;
}

int GlobalApplication::set_usrCfgExposureValues(UINT8 coarseExposure, UINT8 fineExposure, UINT8 grayExposure, UINT8 laserExposurePeriod)
{
    int ret = 0;

    usrCfgCoarseExposure = coarseExposure;
    usrCfgFineExposure = fineExposure;
    usrCfgGrayExposure = grayExposure;
    usrCfgLaserExposurePeriod = laserExposurePeriod;

    return ret;
}

#endif

sensortype GlobalApplication::get_sensor_type()
{
    return selected_sensor_type;
}

int GlobalApplication::set_sensor_type(int sensor_type)
{
    int ret = 0;

    if (SENSOR_TYPE_RGB == sensor_type || SENSOR_TYPE_DTOF == sensor_type)
    {
        selected_sensor_type = (sensortype) sensor_type;
    }
    else {
        ret = -1;
    }

    return ret;
}

int GlobalApplication::get_save_cnt()
{
    return save_frame_cnt;
}

int GlobalApplication::get_timer_test_times()
{
    return timer_test_times;
}

bool GlobalApplication::get_qt_ui_test()
{
    return qt_ui_test;
}

sensortype GlobalApplication::string_2_sensortype(QString& str)
{
    if (!str.compare("RGB"))
        return SENSOR_TYPE_RGB;
    else
        return SENSOR_TYPE_DTOF;
}

#if defined(RUN_ON_EMBEDDED_LINUX)
AdapsEnvironmentType GlobalApplication::string_2_environmenttype(QString& str)
{
    if (!str.compare("Outdoor"))
        return AdapsEnvTypeOutdoor;
    else
        return AdapsEnvTypeIndoor;
}

AdapsMeasurementType GlobalApplication::string_2_measurementtype(QString& str)
{
    if (!str.compare("Short"))
        return AdapsMeasurementTypeShort;
    else if (!str.compare("Normal"))
        return AdapsMeasurementTypeNormal;
    else
        return AdapsMeasurementTypeFull;
}

AdapsPowerMode GlobalApplication::string_2_powermode(QString& str)
{
    if (!str.compare("Div1"))
        return AdapsPowerModeNormal;
    else if (!str.compare("Div3"))
        return AdapsPowerModeDiv3;
    else
        return AdapsPowerModeNormal;
}
#endif

sensor_workmode GlobalApplication::string_2_workmode(QString& str)
{
    if (!str.compare("PCM"))
        return WK_DTOF_PCM;
    else if (!str.compare("FHR"))
        return WK_DTOF_FHR;
    else if (!str.compare("NV12"))
        return WK_RGB_NV12;
    else if (!str.compare("YUYV"))
        return WK_RGB_YUYV;
    else
        return WK_DTOF_PHR;
}
