QT += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = SpadisQT
TEMPLATE = app

# --- 根据编译模式配置选项 ---
CONFIG(debug, debug|release) {
    # Debug 模式：关闭优化，生成调试符号，支持 backtrace 抓取函数名
    message("Configuring for DEBUG mode...")
    QMAKE_CXXFLAGS += -O0 -g -rdynamic
    QMAKE_LFLAGS   += -rdynamic
} else {
    # Release 模式：高优化，启用 LTO，定义 release 宏
    message("Configuring for RELEASE mode...")
    QMAKE_CXXFLAGS += -O3 -flto -fno-exceptions
    QMAKE_LFLAGS   += -flto -Wl,--gc-sections
    DEFINES += BUILD_4_RELEASE
}

DEFINES += RUN_ON_EMBEDDED_LINUX

SOURCES += \
        globalapplication.cpp \
        main.cpp \
        mainwindow.cpp \
        FrameProcessThread.cpp \
        utils.cpp \
        v4l2.cpp

HEADERS += \
        mainwindow.h \
        common.h \
        depthmapwrapper.h \
        globalapplication.h \
        FrameProcessThread.h \
        utils.h \
        v4l2.h

LIBS += -lssl -lcrypto -lz

contains(DEFINES, RUN_ON_EMBEDDED_LINUX) {
    DEFINES += RUN_ON_RK3568
    DEFINES += CONFIG_VIDEO_ADS6401
    DEFINES += ENABLE_POINTCLOUD_OUTPUT
#    DEFINES += ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB
#    DEFINES += STANDALONE_APP_WITHOUT_HOST_COMMUNICATION

    QMAKE_LFLAGS += -Wl,-rpath,/vendor/lib64/
    SOURCES += adaps_dtof.cpp \
               host_comm.cpp \
               misc_device.cpp
    HEADERS += misc_device.h \
               host_comm.h \
               adaps_dtof.h \
               adaps_dtof_uapi.h
    LIBS += -L$$PWD -ladaps_swift_decode -lAdapsSender
}

FORMS += mainwindow.ui