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

int main(int argc, char *argv[])
{
    int result;
    GlobalApplication a(argc, argv);

    QString tempPath = QDir::temp().absoluteFilePath(APP_NAME ".lock");
    QLockFile lockFile(tempPath);
    lockFile.isLocked();

    if (!lockFile.tryLock(100))
    {
        DBG_NOTICE("An instance of the application is already running!");

        return 1;
    }

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

#if !defined(CONSOLE_APP_WITHOUT_GUI)
    w.setWindowFlags(w.windowFlags() & ~Qt::WindowMaximizeButtonHint & ~Qt::WindowMinimizeButtonHint);
    w.show();
#endif

    result = a.exec();

    return result;
}
