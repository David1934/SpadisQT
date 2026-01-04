#include <signal.h>

#if !defined(CONSOLE_APP_WITHOUT_GUI)
#include <QTime>
#include <QTimer>
#include <QShortcut>
//#include <QMessageBox>
#include <QCloseEvent>
#include <QLCDNumber>
#include <QScreen>
#include <QMouseEvent>
#include <QSize>

#include "ui_mainwindow.h"
#endif

#include "globalapplication.h"
#include "common.h"
#include "mainwindow.h"
#include "FrameProcessThread.h"

#define UNUSED(X) (void)X
#define QStringToCharPtr(qstr) (qstr.toUtf8().constData())


#if !defined(CONSOLE_APP_WITHOUT_GUI)
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
#else
MainWindow::MainWindow()
#endif
{
    char AppNameVersion[128];
#if !defined(CONSOLE_APP_WITHOUT_GUI)
    int w;
    int h;
#endif

    firstDisplayFrameTimeUsec = 0;
    displayedFrameCnt = 0;
    displayed_fps = 0;
    frame_process_thread = NULL_POINTER;
#if !defined(CONSOLE_APP_WITHOUT_GUI)
    ui->setupUi(this);
#endif
    sprintf(AppNameVersion, "%s %s built at %s,%s"
        ,APP_NAME,
        APP_VERSION,
        __DATE__, __TIME__
        );
#if !defined(CONSOLE_APP_WITHOUT_GUI)
    this->setWindowTitle(AppNameVersion);
#endif
    DBG_NOTICE("AppVersion: %s, Build on QT version: %s, Runtime lib QT version: %s...",
        AppNameVersion, QT_VERSION_STR, qVersion());

#if !defined(CONSOLE_APP_WITHOUT_GUI)
    w = this->width();
    h = this->height();
    DBG_INFO("MainWindow resolution: %d X %d", w, h);

    // 设置QLCDNumber的显示属性
    ui->rtcTime->setDigitCount(8); // 显示格式为HH:MM:SS
    ui->rtcTime->display(QTime::currentTime().toString(RTCTIME_DISPLAY_FMT));

    // 创建一个定时器，每秒更新时间
    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::updateRTCtime);
    timer->start(1000); // 设置定时器每1000毫秒触发一次

    connect(ui->quitButton, SIGNAL(clicked()), this, SLOT(Quit()));

    if (true == qApp->get_qt_ui_test())
        return ;

    ui->startStopButton->setText("Start");

#if defined(RUN_ON_EMBEDDED_LINUX)
    ui->groupBox_rgb_wkmode->setEnabled(false);
    ui->groupBox_rgb_wkmode->setVisible(false);
#else
    ui->groupBox_dtof_wkmode->setEnabled(false);
    ui->groupBox_dtof_environment->setEnabled(false);
    ui->groupBox_dtof_wkmode->setVisible(false);
    ui->groupBox_dtof_environment->setVisible(false);
    ui->groupBox_dtof_framerate->setVisible(false);
    ui->confidence_view->setVisible(false);
    ui->groupBox_dtof_powermode->setVisible(false);
#endif
#endif

#if defined(RUN_ON_EMBEDDED_LINUX)
    m_misc_dev = new Misc_Device();
    qApp->register_misc_dev_instance(m_misc_dev);
#if !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
    host_comm = Host_Communication::getInstance();

    connect(host_comm, &Host_Communication::start_capture, this, &MainWindow::on_startCapture);
    connect(host_comm, &Host_Communication::stop_capture, this, &MainWindow::on_stopCapture);

    qRegisterMetaType<capture_req_param_t*>("capture_req_param_t*");
    connect(host_comm, &Host_Communication::set_capture_options, this, &MainWindow::on_capture_options_set);
#endif
#endif

    startFrameProcessThread();
}

MainWindow::~MainWindow()
{
    DBG_INFO("frame_process_thread:%p...", frame_process_thread);

    if (NULL_POINTER != frame_process_thread)
    {
        bool isRunning = frame_process_thread->isRunning();
        bool isFinished = frame_process_thread->isFinished();
        bool isSleeping = frame_process_thread->isSleeping();

        DBG_INFO("frame_process_thread isRunning():%d, isFinished(): %d, isSleeping: %d",
            isRunning,
            isFinished,
            isSleeping
            );
        if (frame_process_thread->isRunning())
        {
            frame_process_thread->exit(0);
            //frame_process_thread->wait(WAIT_TIME_4_THREAD_EXIT); // unit is milliseconds
        }
        delete frame_process_thread;
        frame_process_thread = NULL_POINTER;
    }

#if !defined(CONSOLE_APP_WITHOUT_GUI)
    delete ui;
    ui = NULL_POINTER;
#endif
#if defined(RUN_ON_EMBEDDED_LINUX)
#if !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
    delete host_comm;
    host_comm = NULL_POINTER;
#endif
    delete m_misc_dev;
    m_misc_dev = NULL_POINTER;
    qApp->register_misc_dev_instance(m_misc_dev);
#endif
}

void MainWindow::startFrameProcessThread(void)
{
    int ret = 0;
#if !defined(CONSOLE_APP_WITHOUT_GUI)
    char auto_test_times_string[32];
#endif
    sensortype sensor_type = qApp->get_sensor_type();
    sensor_workmode wk_mode = qApp->get_wk_mode();

    if (SENSOR_TYPE_DTOF == sensor_type)
    {
        if (wk_mode >= WK_DTOF_PHR && wk_mode <= WK_DTOF_FHR)
        {
            // okay, continue
        }
        else {
            // No correct work mode is selected, exit.
            return;
        }
    }
    else if (SENSOR_TYPE_RGB == sensor_type)
    {
        if (wk_mode >= WK_RGB_NV12 && wk_mode <= WK_RGB_YUYV)
        {
            // okay, continue
        }
        else {
            // No correct work mode is selected, exit.
            return;
        }
    }
    else {
        // No correct sensor type is selected, exit.
        return;
    }

    frame_process_thread = new FrameProcessThread;
    ret = frame_process_thread->init(0);
    if (ret < 0)
    {
#if defined(RUN_ON_EMBEDDED_LINUX) && !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
        if (host_comm)
        {
            char err_msg[] = "Fail to frame_process_thread init\nPls check device side log for detailed info!";
            host_comm->report_status(CMD_HOST_SIDE_START_CAPTURE, CMD_DEVICE_SIDE_ERROR_FAIL_TO_START_CAPTURE, err_msg, strlen(err_msg));
        }
#endif

        DBG_ERROR("Fail to frame_process_thread init()...");
#if !defined(CONSOLE_APP_WITHOUT_GUI)
        ui->depth_view->setText("Fail to frame_process_thread init(),\nPlease check camera is ready or not?");
#endif
        delete frame_process_thread;
        frame_process_thread = NULL_POINTER;
        //QMessageBox::information(nullptr, "Warning Prompt", "No camera is detected, please double check!");
        return;
    }

    displayedFrameCnt = 0;
    connect(frame_process_thread, SIGNAL(threadEnd(int)), this, SLOT(onThreadEnd(int)));

#if !defined(CONSOLE_APP_WITHOUT_GUI)
    qRegisterMetaType<enum frame_data_type>("enum frame_data_type");
    connect(frame_process_thread, SIGNAL(newFrameReady4Display(unsigned int, QImage, QImage)),
            this, SLOT(new_frame_display(unsigned int, QImage, QImage)));
    qRegisterMetaType<status_params2>("status_params2");
    connect(frame_process_thread, SIGNAL(update_runtime_display(status_params2)),
            this, SLOT(update_status_info(status_params2)));

    // 创建一个QShortcut对象，将Ctrl + C组合键与槽函数onCtrlCPressed关联起来
    QShortcut *shortcut_ctrlX = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_X), this);
    connect(shortcut_ctrlX, &QShortcut::activated, this, &MainWindow::onCtrl_X_Pressed);

    ui->depth_view->installEventFilter(this); // for eventFilter()

    tested_times = 0;
    to_test_times = qApp->get_timer_test_times();
    if (to_test_times > 0)
    {
        test_timer = new QTimer(this);
        connect(test_timer, &QTimer::timeout, this, &MainWindow::simulateButtonClick);
    }

    sprintf(auto_test_times_string, "%d/%d", tested_times, to_test_times);
    ui->AutoTestTimes_value->setText(auto_test_times_string);
#endif

    frame_process_thread->start();

#if !defined(CONSOLE_APP_WITHOUT_GUI)
    if (to_test_times > 0)
    {
        test_timer->start(1000 * TIMER_TEST_INTERVAL);
    }
    ui->startStopButton->setText("Stop");
#endif
}

void MainWindow::onThreadEnd(int stop_request_code)
{
    switch (stop_request_code) {
        case STOP_REQUEST_RGB:
            break;

        case STOP_REQUEST_PHR:
            break;

        case STOP_REQUEST_PCM:
            break;

        case STOP_REQUEST_FHR:
            break;

        case STOP_REQUEST_QUIT:
#if !defined(CONSOLE_APP_WITHOUT_GUI)
            this->close();
            qApp->exit(0); // exit from the application
#endif
            break;

        case STOP_REQUEST_STOP:
            delete frame_process_thread;
            frame_process_thread = NULL_POINTER;
#if !defined(CONSOLE_APP_WITHOUT_GUI)
            ui->depth_view->setText("Device is stopped");
            ui->confidence_view->setText("Confidence bitmap is NA");
#endif
        break;

        default:
#if !defined(CONSOLE_APP_WITHOUT_GUI)
            ui->depth_view->setText("Device is ready");
#endif
            break;
    }

}

void MainWindow::Quit(void)
{
    if (true == qApp->get_qt_ui_test())
    {
        //this->close();
        qApp->exit(0);
    }
    else {
        if (NULL_POINTER != frame_process_thread)
        {
            bool isRunning = frame_process_thread->isRunning();
            bool isFinished = frame_process_thread->isFinished();
            bool isSleeping = frame_process_thread->isSleeping();

            DBG_INFO("frame_process_thread isRunning():%d, isFinished(): %d, isSleeping: %d",
                isRunning,
                isFinished,
                isSleeping
                );
            if (isRunning)
            {
                frame_process_thread->stop(STOP_REQUEST_QUIT);
            }
            else {
                if (isFinished)
                {
#if !defined(CONSOLE_APP_WITHOUT_GUI)
                    this->close(); // frame_process_thread->exit(1);
#else
                    qApp->exit(0);
#endif
                }
            }
        }
        else {
            //this->close();
            qApp->exit(0);
        }
    }
}

void MainWindow::unixSignalHandler(int signal)
{
    switch (signal) {
        case SIGTSTP:
#if !defined(CONSOLE_APP_WITHOUT_GUI)
            DBG_NOTICE("CTRL-Z recieved, screen shot will be excuted!");
            captureAndSaveScreenshot();
#endif
            break;

        case SIGUSR1:
            DBG_ERROR("User Signal 1 recieved, some error happened\nPls check kernel log for detailed info!");
#if !defined(CONSOLE_APP_WITHOUT_GUI)
            ui->depth_view->setText("User Signal 1 recieved, some error happened\nPls check kernel log for detailed info!");
#endif
            if (NULL_POINTER != frame_process_thread)
            {
                frame_process_thread->stop(STOP_REQUEST_STOP);
            }
#if defined(RUN_ON_EMBEDDED_LINUX) && !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
            if (host_comm)
            {
                char err_msg[] = "User Signal 1 recieved, some error happened\nPls check kernel log for detailed info!";
                host_comm->report_status(CMD_HOST_SIDE_START_CAPTURE, CMD_DEVICE_SIDE_ERROR_CAPTURE_ABORT, err_msg, strlen(err_msg));
            }
#endif
            break;

        case SIGUSR2:
            DBG_ERROR("User Signal 2 recieved, some error happened\nPls check kernel log for detailed info!");
#if !defined(CONSOLE_APP_WITHOUT_GUI)
            ui->depth_view->setText("User Signal 2 recieved, some error happened\nPls check kernel log for detailed info!");
#endif
            if (NULL_POINTER != frame_process_thread)
            {
                frame_process_thread->stop(STOP_REQUEST_STOP);
            }
#if defined(RUN_ON_EMBEDDED_LINUX) && !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
            if (host_comm)
            {
                char err_msg[] = "User Signal 2 recieved, some error happened\nPls check kernel log for detailed info!";
                host_comm->report_status(CMD_HOST_SIDE_START_CAPTURE, CMD_DEVICE_SIDE_ERROR_CAPTURE_ABORT, err_msg, strlen(err_msg));
            }
#endif
            break;

        case SIGINT:
            DBG_NOTICE("CTRL-C recieved, graceful close() is executed!");
#if defined(RUN_ON_EMBEDDED_LINUX) && !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
            if (host_comm)
            {
                char err_msg[] = "The device-side application was terminated by the user.\nStream capture is aborted!";
                host_comm->report_status(CMD_HOST_SIDE_START_CAPTURE, CMD_DEVICE_SIDE_ERROR_CAPTURE_ABORT, err_msg, strlen(err_msg));
            }
#endif
            Quit();
            break;

        case SIGSEGV:
        case SIGBUS:
        case SIGTERM:
            DBG_ERROR("graceful close() is executed!");
#if defined(RUN_ON_EMBEDDED_LINUX) && !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
            if (host_comm)
            {
                char err_msg[] = "some crytical error happened\nStream capture is aborted!";
                host_comm->report_status(CMD_HOST_SIDE_START_CAPTURE, CMD_DEVICE_SIDE_ERROR_CAPTURE_ABORT, err_msg, strlen(err_msg));
            }
#endif
            Quit();
            break;

        default:
            break;
    }

    return;
}

void MainWindow::on_stopCapture()
{
    if (NULL_POINTER != frame_process_thread)
    {
        frame_process_thread->stop(STOP_REQUEST_STOP);
#if !defined(CONSOLE_APP_WITHOUT_GUI)
        ui->startStopButton->setText("Start");
#endif
    }
}

#if defined(RUN_ON_EMBEDDED_LINUX) && !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
void MainWindow::on_capture_options_set(capture_req_param_t* param)
{
    switch (param->work_mode) 
    {
#if !defined(CONSOLE_APP_WITHOUT_GUI)
        case ADAPS_PCM_MODE:
            ui->radioButton_pcm->setChecked(true);
            break;

        case ADAPS_PTM_PHR_MODE:
            ui->radioButton_phr->setChecked(true);
            break;

        default:
            ui->radioButton_fhr->setChecked(true);
            break;
#else
        case ADAPS_PCM_MODE:
            qApp->set_wk_mode(WK_DTOF_PCM);
            break;

        case ADAPS_PTM_PHR_MODE:
            qApp->set_wk_mode(WK_DTOF_PHR);
            break;

        default:
            qApp->set_wk_mode(WK_DTOF_FHR);
            break;
#endif
    }

    switch (param->power_mode) 
    {
#if 0 //!defined(CONSOLE_APP_WITHOUT_GUI)
        case AdapsPowerModeDiv3:
            ui->radioButton_pmode_div3->setChecked(true);
            break;

        default:
            ui->radioButton_pmode_div1->setChecked(true);
            break;
#else
        case AdapsPowerModeDiv3:
            qApp->set_power_mode(AdapsPowerModeDiv3);
            break;

        default:
            qApp->set_power_mode(AdapsPowerModeNormal);
            break;
#endif
    }

    switch (param->env_type) 
    {
#if !defined(CONSOLE_APP_WITHOUT_GUI)
        case AdapsEnvTypeOutdoor:
            ui->radioButton_outdoor->setChecked(true);
            break;

        default:
            ui->radioButton_indoor->setChecked(true);
            break;
#else
        case AdapsEnvTypeOutdoor:
            qApp->set_environment_type(AdapsEnvTypeOutdoor);
            break;

        default:
            qApp->set_environment_type(AdapsEnvTypeIndoor);
            break;
#endif
    }

    switch (param->framerate_type) 
    {
#if !defined(CONSOLE_APP_WITHOUT_GUI)
        case AdapsFramerateType15FPS:
            ui->radioButton_15fps->setChecked(true);
            break;

        case AdapsFramerateType25FPS:
            ui->radioButton_25fps->setChecked(true);
            break;

        case AdapsFramerateType60FPS:
            ui->radioButton_60fps->setChecked(true);
            break;

        case AdapsFramerateType30FPS:
        default:
            ui->radioButton_30fps->setChecked(true);
            break;
#else
        case AdapsFramerateType15FPS:
            qApp->set_framerate_type(AdapsFramerateType15FPS);
            break;

        case AdapsFramerateType25FPS:
            qApp->set_framerate_type(AdapsFramerateType25FPS);
            break;

        case AdapsFramerateType60FPS:
            qApp->set_framerate_type(AdapsFramerateType60FPS);
            break;

        case AdapsFramerateType30FPS:
        default:
            qApp->set_framerate_type(AdapsFramerateType30FPS);
            break;
#endif
    }
}
#endif

void MainWindow::on_startCapture()
{
    if (NULL_POINTER == frame_process_thread)
    {
#if !defined(CONSOLE_APP_WITHOUT_GUI)
        sensortype sensor_type = qApp->get_sensor_type();

#if defined(RUN_ON_EMBEDDED_LINUX)
        if (SENSOR_TYPE_DTOF == sensor_type)
        {
            if(ui->radioButton_fhr->isChecked()) {
                qApp->set_wk_mode(WK_DTOF_FHR);
            } else if(ui->radioButton_pcm->isChecked()) {
                qApp->set_wk_mode(WK_DTOF_PCM);
            } else {
                qApp->set_wk_mode(WK_DTOF_PHR);
            }

#if 0
            if(ui->radioButton_pmode_div3->isChecked()) {
                qApp->set_power_mode(AdapsPowerModeDiv3);
            } else {
                qApp->set_power_mode(AdapsPowerModeNormal);
            }
#endif

            if(ui->radioButton_outdoor->isChecked()) {
                qApp->set_environment_type(AdapsEnvTypeOutdoor);
            } else {
                qApp->set_environment_type(AdapsEnvTypeIndoor);
            }

            if(ui->radioButton_15fps->isChecked()) {
                qApp->set_framerate_type(AdapsFramerateType15FPS);
            } else if(ui->radioButton_25fps->isChecked()) {
                qApp->set_framerate_type(AdapsFramerateType25FPS);
            } else if(ui->radioButton_60fps->isChecked()) {
                qApp->set_framerate_type(AdapsFramerateType60FPS);
            } else {
                qApp->set_framerate_type(AdapsFramerateType30FPS);
            }
        }
        else 
#endif
        if (SENSOR_TYPE_RGB == sensor_type)
        {
            if(ui->radioButton_yuyv->isChecked()) {
                qApp->set_wk_mode(WK_RGB_YUYV);
            } else {
                qApp->set_wk_mode(WK_RGB_NV12);
            }
        }
        else {
            // No correct sensor type is selected, exit.
            return;
        }
#endif

#if defined(RUN_ON_EMBEDDED_LINUX)
#if !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
        if (false == qApp->is_capture_req_from_host())
#endif
        {
            // For flood module on Hisilicon platform, PCM mode always use Div3 since too high temperature
            if (ADS6401_MODULE_SMALL_FLOOD == qApp->get_module_type()
                 && WK_DTOF_PCM == qApp->get_wk_mode()
                )
            {
                qApp->set_power_mode(AdapsPowerModeDiv3);
            }
        }
#endif

        //qApp->set_wk_mode(WK_DTOF_FHR);
        startFrameProcessThread();
    }
}


#if !defined(CONSOLE_APP_WITHOUT_GUI)
bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    if (obj == ui->depth_view && event->type() == QEvent::MouseButtonPress)
    {
        QSize labelSize = ui->depth_view->size();
        QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
        QPoint pos = mouseEvent->pos();
        DBG_INFO("pos (%d, %d) label.size(): %d X %d", 
            pos.x(), pos.y(), labelSize.width(), labelSize.height());
        //frame_process_thread->setWatchSpot(labelSize, pos);

        return true;
    }
    return QMainWindow::eventFilter(obj, event);
}

void MainWindow::updateRTCtime()
{
    // 获取当前时间，并更新QLCDNumber控件显示
    QTime currentTime = QTime::currentTime();
    ui->rtcTime->display(currentTime.toString(RTCTIME_DISPLAY_FMT));
}

void MainWindow::onCtrl_X_Pressed(void)
{
    DBG_INFO("Ctrl + X was pressed.");
    Quit();
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    DBG_INFO("Last chance to do something before exit...");

    // 在这里执行退出前的处理逻辑
    // 例如保存数据、释放资源等

    // 接受关闭事件
    event->accept();
}

bool MainWindow::update_status_info(status_params2 param2)
{
    char temp_string[32];
    const char *work_mode_name[]={
        "PTM-PHR",
        "PCM-Gray",
        "PTM-FHR",
        "RGB-NV12",
        "RGB-YUYV",
    };

#if defined(RUN_ON_EMBEDDED_LINUX)
    const char *environment_type_name[]={
        "Unknown",
        "Indoor",
        "Outdoor",
    };

    const char *measurement_range_name[]={
        "Unknown",
        "Normal",
        "Short",
        "Full",
    };

    const char *power_mode_names[] =
    {
        "Unknown",
        "Div1",
        "Div2",
        "Div3",
    };

#endif

    unsigned int streamed_time_seconds = param2.streamed_time_us / 1000000;
#if 0
    DBG_NOTICE("---------frm_sequence: %d, displayedFrameCnt: %d, displayed_fps: %d, mipi_rx_fps:%d ---\n",
        param2.frm_sequence, displayedFrameCnt, displayed_fps, param2.mipi_rx_fps);
#endif
    sprintf(temp_string, "%d (%d) fps", displayed_fps, param2.mipi_rx_fps);
    ui->framerate_value->setText(temp_string);

    sprintf(temp_string, "%02d:%02d:%02d", streamed_time_seconds/3600, streamed_time_seconds/60, streamed_time_seconds%60);
    ui->strmTime_value->setText(temp_string);

    if (param2.work_mode >= WK_DTOF_PHR && param2.work_mode < WK_COUNT)
    {
        ui->cur_work_mode_value->setText(work_mode_name[param2.work_mode]);
    }
    else {
        ui->cur_work_mode_value->setText("Unknown");
    }

#if defined(RUN_ON_EMBEDDED_LINUX)
    if (SENSOR_TYPE_RGB == param2.sensor_type)
    {
        ui->cur_module_type_value->setText("RGB");
    }
    else {
        if (ADS6401_MODULE_SPOT == qApp->get_module_type())
        {
            ui->cur_module_type_value->setText(SPOT_MODULE_TYPE_NAME);
        }
        else if (ADS6401_MODULE_SMALL_FLOOD == qApp->get_module_type()) {
            ui->cur_module_type_value->setText(FLOOD_MODULE_TYPE_NAME);
            ui->pvddLabel->setText("Cur expected vbat:");
        }
        else if (ADS6401_MODULE_BIG_FOV_FLOOD_V2 == qApp->get_module_type()) {
            ui->cur_module_type_value->setText(BIG_FOV_FLOOD_V2_MODULE_TYPE_NAME);
        }
        else {
            ui->cur_module_type_value->setText(BIG_FOV_FLOOD_MODULE_TYPE_NAME);
        }

        sprintf(temp_string, "%d.%02d V", param2.curr_exp_pvdd/100, param2.curr_exp_pvdd%100);
        ui->cur_exp_pvdd_value->setText(temp_string);
    }

    sprintf(temp_string, "%d.%02d ℃", param2.curr_temperature/100, param2.curr_temperature%100);
    ui->cur_temperature_value->setText(temp_string);

    sprintf(temp_string, "-%d.%02d V", param2.curr_exp_vop_abs/100, param2.curr_exp_vop_abs%100);
    ui->cur_exp_vop_value->setText(temp_string);

    if (param2.env_type >= AdapsEnvTypeIndoor && param2.env_type <= AdapsEnvTypeOutdoor)
    {
        ui->cur_environment_value->setText(environment_type_name[param2.env_type]);
    }
    else {
        ui->cur_environment_value->setText("Unknown");
    }

    if (param2.curr_power_mode >= AdapsPowerModeNormal && param2.curr_power_mode <= AdapsPowerModeDiv3)
    {
        ui->cur_powermode_value->setText(power_mode_names[param2.curr_power_mode]);
    }
    else {
        ui->cur_powermode_value->setText("Unknown");
    }

    if (param2.measure_type >= AdapsMeasurementTypeNormal && param2.measure_type <= AdapsMeasurementTypeFull)
    {
        ui->cur_measurement_value->setText(measurement_range_name[param2.measure_type]);
    }
    else {
        ui->cur_measurement_value->setText("Unknown");
    }
#else
    ui->cur_module_type_value->setText("RGB camera");
    sprintf(temp_string, "%s", "NA");
    ui->cur_temperature_value->setText(temp_string);
    ui->cur_exp_vop_value->setText(temp_string);
    ui->cur_exp_pvdd_value->setText(temp_string);
    ui->cur_environment_value->setText(temp_string);
    ui->cur_measurement_value->setText(temp_string);

#endif

    // disable the current mode button
    switch (param2.work_mode) {
        case WK_DTOF_PHR:
            break;

        case WK_DTOF_FHR:
             break;

        case WK_DTOF_PCM:
            break;

        case WK_RGB_NV12:
        case WK_RGB_YUYV:
            break;

        default:
            break;
    }

    return true;
}

bool MainWindow::new_frame_display(unsigned int frm_sequence, QImage image, QImage img4confidence)
{
    struct timeval tv;
    long currTimeUsec;
    unsigned long streamed_timeUs;

    ui->depth_view->clear();
    if(image.isNull())
    {
        ui->depth_view->setText("NO IMAGE TO BE DISPLAYED!");
    }
    else
    {
        image = image.scaled(ui->depth_view->width(),ui->depth_view->height(),Qt::IgnoreAspectRatio,Qt::SmoothTransformation);
        QPixmap pix = QPixmap::fromImage(image,Qt::AutoColor);
        ui->depth_view->setPixmap(pix);

        if(false == img4confidence.isNull())
        {
            img4confidence = img4confidence.scaled(ui->confidence_view->width(),ui->confidence_view->height(),Qt::IgnoreAspectRatio,Qt::SmoothTransformation);
            QPixmap pix = QPixmap::fromImage(img4confidence,Qt::AutoColor);
            ui->confidence_view->setPixmap(pix);
        }
        else {
            ui->confidence_view->setText("Confidence bitmap is NA");
        }

        gettimeofday(&tv,NULL);
        displayedFrameCnt++;
        if (1 == displayedFrameCnt)
        {
            firstDisplayFrameTimeUsec = tv.tv_sec*1000000 + tv.tv_usec;
        }
        else {
            currTimeUsec = tv.tv_sec*1000000 + tv.tv_usec;
            streamed_timeUs = (currTimeUsec - firstDisplayFrameTimeUsec);
            displayed_fps = (displayedFrameCnt * 1000000) / streamed_timeUs;
//            DBG_NOTICE("---------frm_sequence: %d, displayedFrameCnt: %d, streamed_timeUs:%d, currTimeUsec: %d (%lld), tv_sec: %d---\n",
//                frm_sequence, displayedFrameCnt, streamed_timeUs, currTimeUsec, currTimeUsec, tv.tv_sec);
        }
    }

    return true;
}

void MainWindow::on_startStopButton_clicked()
{
    if (NULL_POINTER != frame_process_thread)
    {
        on_stopCapture();
    }
    else {
        on_startCapture();
    }
}

void MainWindow::simulateButtonClick()
{
    char auto_test_times_string[32];

    if (tested_times >= to_test_times)
    {
        DBG_NOTICE("\n------Timer test done------test times: %d/%d---\n",
            tested_times, to_test_times);
        test_timer->stop();
        return;
    }

    DBG_NOTICE("\n------Timer testing------test times: %d/%d, isRunning: %d---\n", tested_times, to_test_times, frame_process_thread->isRunning());
    if (NULL_POINTER != frame_process_thread)
    {
        if(frame_process_thread->isRunning())
        {
            tested_times++;
            sprintf(auto_test_times_string, "%d/%d", tested_times, to_test_times);
            ui->AutoTestTimes_value->setText(auto_test_times_string);

            on_startStopButton_clicked();
        }
        else {
            on_startStopButton_clicked();
        }
    }
}

void MainWindow::captureAndSaveScreenshot()
{
    //QList<QByteArray> formats = QImageWriter::supportedImageFormats();
    //qCritical() << "Supported image formats:" << formats;

    // 获取当前窗口的屏幕
    QScreen *screen = QGuiApplication::primaryScreen();
    if (!screen) {
        DBG_ERROR("Failed to get screen object.");
        return;
    }

    // 截取当前窗口的截图
    QPixmap screenshot = screen->grabWindow(winId());

    // 获取当前进程ID
    qint64 processId = QCoreApplication::applicationPid();

    // 获取当前日期和时间
    QDateTime currentDateTime = QDateTime::currentDateTime();
    QString dateStr = currentDateTime.toString("yyyyMMdd");
    QString timeStr = currentDateTime.toString("HHmmss");

    // 构建保存路径
    QString savePath = QString(DATA_SAVE_PATH "Screenshot_4_process%1_%2_%3." CAPTURED_PICTURE_FMT)
                           .arg(processId)
                           .arg(dateStr)
                           .arg(timeStr);

    // 保存截图到文件
    if (screenshot.save(savePath, CAPTURED_PICTURE_FMT)) {
        DBG_NOTICE("Save screenshot to %s successfully.", QStringToCharPtr(savePath));
    } else {
        DBG_ERROR("Failed to save screenshot to %s.", QStringToCharPtr(savePath));
    }
}

#endif

