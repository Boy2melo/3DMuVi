#-------------------------------------------------
#
# Project created by QtCreator 2016-02-15T22:18:43
#
#-------------------------------------------------

QT       += core gui widgets

TARGET = PluginSurf
TEMPLATE = lib
CONFIG += plugin
QMAKE_CXXFLAGS += --std=c++11

CONFIG += link_pkgconfig
PKGCONFIG += opencv

SOURCES += \
    plugin.cpp \
    algorithm.cpp \
    MyAlgorithm.cpp \
    surfmatch-algorithm.cpp

INCLUDEPATH += ../../../3DMuVi \
    OpenSURFcpp/src

DEFINES += BUILD_NAME=\\\"plugins/$${TARGET}\\\"

HEADERS += \
    ../../../3DMuVi/workflow/plugin/ialgorithm.h \
    ../../../3DMuVi/workflow/plugin/iplugin.h \
    plugin.h \
    algorithm.h \
    plugin_config.h \
    surfmatch-algorithm.h
OTHER_FILES += \
    Plugin.json

unix {
    target.path = /usr/lib
    INSTALLS += target
}
