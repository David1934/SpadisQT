#if !defined(CONSOLE_APP_WITHOUT_GUI)
#include <QScreen>
#include <QDesktopWidget>
#include <QMessageBox>
#endif

#include <QLockFile>
#include <csignal>
#include <unistd.h>
#include <execinfo.h>

#include "mainwindow.h"
#include "globalapplication.h"

#define DEPTH_LIB_SO_PATH                   "/vendor/lib64/libadaps_swift_decode.so"

static MainWindow* mainWindow = nullptr;

static void dump_stack()
{
    const int len = 1024;
    void *func[len];
    int size;
    int i;
    char **funs;

    size = backtrace(func, len);
    funs = (char**)backtrace_symbols(func, size);
    if (funs == NULL) {
        DBG_ERROR("backtrace_symbols() fail.\n");
        return;
    }

    DBG_NOTICE("=========> stack trace: %d <=========", size);
    for(i = 0; i < size; ++i)
        DBG_NOTICE("%d %s", i, funs[i]);

    free(funs);
}

static void handleSignal(int signal)
{
    if (mainWindow) {
        switch (signal) {
            case SIGTSTP:
            case SIGUSR1:
            case SIGUSR2:
            case SIGINT:
            case SIGABRT:
                //DBG_NOTICE("Catch Unix signal %d...", signal);
                std::cout << "Received Linux signal: " << signal << std::endl;
                // 将信号编号发送给 MainWindow
                QMetaObject::invokeMethod(mainWindow, "unixSignalHandler", Qt::QueuedConnection, Q_ARG(int, signal));
                break;

            default:
                std::signal(signal, SIG_DFL); // restore this signal to default handle
                //DBG_ERROR("Signal %d recieved, Call stack is printed!", signal);
                std::cout << "Received Linux signal: " << signal << ", Call stack is printed!" << std::endl;
                dump_stack();
                // do nothing more to keep the last state.
                break;
        }
    }
}

#if !defined(CONSOLE_APP_WITHOUT_GUI)
static void getScreenResolution()
{
    qreal screenWidth = QGuiApplication::primaryScreen()->geometry().width();
    qreal screenHeight = QGuiApplication::primaryScreen()->geometry().height();

    qCritical() << "QGuiApplication Screen Resolution:" << screenWidth << "x" << screenHeight;
    DBG_INFO("QGuiApplication Screen  resolution: %f X %f...\n", screenWidth, screenHeight);

    QRect screenGeometry = QApplication::desktop()->geometry();
    int screenWidth2 = screenGeometry.width();
    int screenHeight2 = screenGeometry.height();

    //qCritical() << "QApplication Screen Resolution:" << screenWidth2 << "x" << screenHeight2;
    DBG_INFO("QApplication Screen  resolution: %d X %d...\n", screenWidth2, screenHeight2);
}
#endif

int requirement_check()
{
    int ret = 0;
    Utils *utils;
    utils = new Utils();

    ret = utils->check_regular_file_exist(DEPTH_LIB_SO_PATH);
    if (ret < 0)
    {
        DBG_ERROR("Adaps algo lib <%s> does not exist, please check it!\n", DEPTH_LIB_SO_PATH);
        goto error_exit;
    }

    ret = utils->check_regular_file_exist(DEPTH_LIB_CONFIG_PATH);
    if (ret < 0)
    {
        DBG_ERROR("Adaps algo lib config file <%s> does not exist, please check it!\n", DEPTH_LIB_CONFIG_PATH);
        goto error_exit;
    }

    if (true == Utils::is_env_var_true(ENV_VAR_ENABLE_ALGO_LIB_DUMP_DATA))
    {
        ret = utils->check_dir_exist_and_writable(DEPTH_LIB_DATA_DUMP_PATH);
        if (ret < 0)
        {
            DBG_ERROR("dump directory <%s> does not exist or not writable, please check it!\n", DEPTH_LIB_DATA_DUMP_PATH);
            goto error_exit;
        }
    }

error_exit:
    delete utils;

    return ret;
}

int main(int argc, char *argv[])
{
    int result;
    GlobalApplication a(argc, argv);

    QString tempPath = QDir::temp().absoluteFilePath(APP_NAME ".lock");
    QLockFile lockFile(tempPath);
    lockFile.isLocked();

    if (!lockFile.tryLock(100))
    {
#if 0 //!defined(CONSOLE_APP_WITHOUT_GUI)
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.setText("The application is already running.\n"
            "Allowed to run only one instance of the application.");
        msgBox.exec();
#endif
        DBG_NOTICE("An instance of the application is already running!");

        return 1;
    }

    result =requirement_check();
    if (result < 0)
        return result;

    MainWindow w;
    mainWindow = &w;

    // 注册信号处理函数
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    std::signal(SIGSEGV, handleSignal);
    std::signal(SIGABRT, handleSignal);
    std::signal(SIGBUS, handleSignal);
    std::signal(SIGUSR1, handleSignal);
    std::signal(SIGUSR2, handleSignal);
    std::signal(SIGTSTP, handleSignal);
    DBG_INFO("sizeof(BOOL): %ld, sizeof(float): %ld, sizeof(double): %ld, sizeof(long): %ld, sizeof(long long): %ld", sizeof(BOOL), sizeof(float), sizeof(double), sizeof(long), sizeof(long long));

#if !defined(CONSOLE_APP_WITHOUT_GUI)
    w.setWindowFlags(w.windowFlags() & ~Qt::WindowMaximizeButtonHint & ~Qt::WindowMinimizeButtonHint);
    //w.setStyleSheet("QMainWindow::title { font-family: Arial; font-size: 21pt; }");
    w.show();
    getScreenResolution();
#endif

    result = a.exec();
    DBG_INFO("Application exited with code:: %d", result);

    return result;
}
