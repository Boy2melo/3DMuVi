#-------------------------------------------------
#
# Project created by QtCreator 2016-02-15T22:18:43
#
#-------------------------------------------------

QT       += core gui

TARGET = PluginExample
TEMPLATE = lib
CONFIG += plugin c++11

DESTDIR = $$[QT_INSTALL_PLUGINS]/generic

SOURCES += exampleplugin.cpp \
    examplealgorithm.cpp

INCLUDEPATH += ../../../3DMuVi

DEFINES += BUILD_NAME=\\\"plugins/$${TARGET}\\\"

HEADERS += exampleplugin.h \
    ../../../3DMuVi/workflow/plugin/ialgorithm.h \
    ../../../3DMuVi/workflow/plugin/iplugin.h \
    examplealgorithm.h
OTHER_FILES += PluginExample.json

unix {
    target.path = /usr/lib
    INSTALLS += target
}
