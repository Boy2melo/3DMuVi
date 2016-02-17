#-------------------------------------------------
#
# Project created by QtCreator 2016-02-15T22:18:43
#
#-------------------------------------------------

QT       += core gui

TARGET = PluginSurf
TEMPLATE = lib
CONFIG += plugin c++11

CONFIG += link_pkgconfig
PKGCONFIG += opencv

DESTDIR = $$[QT_INSTALL_PLUGINS]/generic

SOURCES += surfplugin.cpp \
    surfalgorithm.cpp \
    surfmatch-algorithm.cpp

INCLUDEPATH += ../../../3DMuVi \
    OpenSURFcpp/src

DEFINES += BUILD_NAME=\\\"plugins/$${TARGET}\\\"

HEADERS += surfplugin.h \
    ../../../3DMuVi/workflow/plugin/ialgorithm.h \
    ../../../3DMuVi/workflow/plugin/iplugin.h \
    surfalgorithm.h \
    surfmatch-algorithm.h
OTHER_FILES += PluginSurf.json

unix {
    target.path = /usr/lib
    INSTALLS += target
}
