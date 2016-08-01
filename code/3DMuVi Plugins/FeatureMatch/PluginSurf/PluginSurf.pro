#-------------------------------------------------
#
# Project created by QtCreator 2016-02-15T22:18:43
#
#-------------------------------------------------

QT       += core gui widgets

TARGET = PluginSurf
DESTDIR = ../../../Win32/Debug/plugins

TEMPLATE = lib
CONFIG += plugin
QMAKE_CXXFLAGS += --std=c++11

CONFIG += link_pkgconfig
PKGCONFIG += opencv

LIBS += $$PWD/OpenSURFcpp/libsurf.a

SOURCES += \
    MyAlgorithm.cpp \
    surfmatch-algorithm.cpp \
    pluginsurf.cpp

INCLUDEPATH += ../../../3DMuVi \
    OpenSURFcpp/src

DEFINES += BUILD_NAME=\\\"plugins/$${TARGET}\\\"

HEADERS += \
    ../../../3DMuVi/workflow/plugin/ialgorithm.h \
    surfmatch-algorithm.h \
    pluginsurf.h
OTHER_FILES += \
    Plugin.json

unix {
    target.path = /usr/lib
    INSTALLS += target
}
