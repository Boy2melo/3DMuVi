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

HEADERS += gui/CTestCLogWidget.h \
           io/CTestCImageIo.h \
           io/CTestCTextIo.h \
           ../gui/CLogWidget.h \
           ../io/CImageIo.h \
           ../io/CTextIo.h \
           ../logger/controll/CLogController.h \
           ../logger/controll/CLogHistory.h \
           ../logger/qslogging/QsLog.h \
           ../logger/qslogging/QsLogDest.h \
           ../logger/qslogging/QsLogDestConsole.h \
           ../logger/qslogging/QsLogDestFile.h \
           ../logger/qslogging/QsLogDestFunctor.h \
           ../logger/qslogging/QsLogLevel.h

SOURCES += gui/CTestCLogWidget.cpp \
           io/CTestCImageIo.cpp \
           io/CTestCTextIo.cpp \
           ../io/CImageIo.cpp \
           ../io/CTextIo.cpp \
           ../gui/CLogWidget.cpp \
           ../logger/controll/CLogController.cpp \
           ../logger/controll/CLogHistory.cpp \
           ../logger/qslogging/QsLog.cpp \
           ../logger/qslogging/QsLogDest.cpp \
           ../logger/qslogging/QsLogDestConsole.cpp \
           ../logger/qslogging/QsLogDestFile.cpp\
           ../logger/qslogging/QsLogDestFunctor.cpp \
           main.cpp


DISTFILES += \
    io/data/testImage.png \
    io/data/testText.txt

RESOURCES += \
    test.qrc
