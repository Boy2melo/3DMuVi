#-------------------------------------------------
#
# Project created by QtCreator 2016-02-15T22:18:43
#
#-------------------------------------------------

QT       += core gui

TARGET = PluginPoseEstimator
TEMPLATE = lib
CONFIG += plugin

DESTDIR = $$[QT_INSTALL_PLUGINS]/generic

QMAKE_CXXFLAGS += -std=c++11
QMAKE_CXXFLAGS_RELEASE += -O3

# add suffix d to debug output
CONFIG(debug, debug|release) {
  win32: TARGET =  $$join(TARGET,,,d)
  unix: TARGET = $$join(TARGET,,,_debug)
}

SOURCES += \
    plugin.cpp \
    algorithm.cpp \
    MyAlgorithm.cpp

INCLUDEPATH += ../../3DMuVi

DEFINES += BUILD_NAME=\\\"plugins/$${TARGET}\\\"

HEADERS += \
    ../../3DMuVi/workflow/plugin/ialgorithm.h \
    ../../3DMuVi/workflow/plugin/iplugin.h \
    plugin.h \
    algorithm.h \
    plugin_config.h
OTHER_FILES += \
    Plugin.json

unix {
    target.path = /usr/lib
    INSTALLS += target
}
