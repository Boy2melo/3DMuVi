#-------------------------------------------------
#
# Project created by QtCreator 2016-01-30T00:14:16
#
#-------------------------------------------------

QT       += testlib gui widgets

TARGET = test
CONFIG   += console
CONFIG   -= app_bundle

CONFIG += c++11

TEMPLATE = app

INCLUDEPATH += ../

HEADERS += ../gui/CLogWidget.h \
           ../io/CImageIo.h \
           ../io/CTextIo.h

SOURCES += gui/CTestCLogWidget.cpp \
           io/CTestCImageIo.cpp \
           io/CTestCTextIo.cpp \
           ../io/CImageIo.cpp \
           ../io/CTextIo.cpp \
           ../gui/CLogWidget.cpp

DISTFILES += \
    io/data/testImage.png \
    io/data/testText.txt

RESOURCES += \
    test.qrc
