#-------------------------------------------------
#
# Project created by QtCreator 2016-02-15T22:18:43
#
#-------------------------------------------------

QT       += core gui widgets

TARGET = PluginPoseEstimator
TEMPLATE = lib
CONFIG += plugin

#DESTDIR = $$[QT_INSTALL_PLUGINS]/generic
DESTDIR = ../../Win32/Debug/plugins

QMAKE_CXXFLAGS += -std=c++11
QMAKE_CXXFLAGS_RELEASE += -O3

# add suffix d to debug output
CONFIG(debug, debug|release) {
  win32: TARGET =  $$join(TARGET,,,d)
  unix: TARGET = $$join(TARGET,,,_debug)
}

SOURCES += \
    MyAlgorithm.cpp \
    cpluginposeestimator.cpp

INCLUDEPATH += ../../3DMuVi

DEFINES += BUILD_NAME=\\\"plugins/$${TARGET}\\\"

HEADERS += \
    ../../3DMuVi/workflow/plugin/ialgorithm.h \
    cpluginposeestimator.h

OTHER_FILES += \
    PluginPoseEstimator.json

unix {
    target.path = /usr/lib
    INSTALLS += target
}
