#-------------------------------------------------
#
# Project created by QtCreator 2016-01-30T00:14:16
#
#-------------------------------------------------

QT       += testlib gui widgets

TARGET = test
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

INCLUDEPATH += ../

HEADERS += ../gui/CLogWidget.h \
           ../io/CImageIo.h

SOURCES += gui/CTestCLogWidget.cpp \
           io/CTestCImageIo.cpp \
           ../io/CImageIo.cpp \
           ../gui/CLogWidget.cpp
