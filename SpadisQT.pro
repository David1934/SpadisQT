QT += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = SpadisQT
TEMPLATE = app

QMAKE_CXXFLAGS_RELEASE += -O3 -flto -fno-exceptions
QMAKE_LFLAGS_RELEASE   += -flto -Wl,--gc-sections

DEFINES += RUN_ON_EMBEDDED_LINUX

SOURCES +=\
        globalapplication.cpp \
        main.cpp \
        mainwindow.cpp \
        FrameProcessThread.cpp \
        utils.cpp \
        v4l2.cpp

HEADERS += mainwindow.h \
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
#    DEFINES += ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB
#    DEFINES += STANDALONE_APP_WITHOUT_HOST_COMMUNICATION

    QMAKE_LFLAGS += -Wl,-rpath,/vendor/lib64/
    SOURCES += adaps_dtof.cpp
    SOURCES += host_comm.cpp
    SOURCES += misc_device.cpp
    HEADERS += misc_device.h
    HEADERS += host_comm.h
    HEADERS += adaps_dtof.h
    HEADERS += adaps_dtof_uapi.h
    LIBS += -L$$PWD -ladaps_swift_decode -lAdapsSender
}

FORMS    += mainwindow.ui
