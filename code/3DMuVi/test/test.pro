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
           gui/CTestCMainWindow.h \
           gui/CTestCSettingsDialog.h \
           io/CTestCImageIo.h \
           io/CTestCTextIo.h \
           ../gui/CAlgorithmSelector.h \
           ../gui/AlgorithmSettings/CAlgorithmSettingsView.h \
           ../gui/CImagePreviewItem.h \
           ../gui/CImagePreviewWidget.h \
           ../gui/CLogWidget.h \
           ../gui/CMainWindow.h \
           ../gui/CSettingsDialog.h \
           ../gui/CStepComboBox.h \
           ../io/CImageIo.h \
           ../io/CInputDataSet.h \
           ../io/CTextIo.h \
           ../logger/controll/CLogController.h \
           ../logger/controll/CLogHistory.h \
           ../logger/qslogging/QsLog.h \
           ../logger/qslogging/QsLogDest.h \
           ../logger/qslogging/QsLogDestConsole.h \
           ../logger/qslogging/QsLogDestFile.h \
           ../logger/qslogging/QsLogDestFunctor.h \
           ../logger/qslogging/QsLogLevel.h \
           ../settings/CGlobalSettingController.h \
           ../workflow/workflow/aworkflow.h \
           ../workflow/workflow/cworkflowmanager.h \
           ../workflow/plugin/cpluginmanager.h

SOURCES += gui/CTestCLogWidget.cpp \
           gui/CTestCMainWindow.cpp \
           gui/CTestCSettingsDialog.cpp \
           io/CTestCImageIo.cpp \
           io/CTestCTextIo.cpp \
           ../gui/CAlgorithmSelector.cpp \
           ../gui/AlgorithmSettings/CAlgorithmSettingsView.cpp \
           ../gui/CImagePreviewItem.cpp \
           ../gui/CImagePreviewWidget.cpp \
           ../gui/CLogWidget.cpp \
           ../gui/CMainWindow.cpp \
           ../gui/CSettingsDialog.cpp \
           ../gui/CStepComboBox.cpp \
           ../io/CImageIo.cpp \
           ../io/CInputDataSet.cpp \
           ../io/CTextIo.cpp \
           ../logger/controll/CLogController.cpp \
           ../logger/controll/CLogHistory.cpp \
           ../logger/qslogging/QsLog.cpp \
           ../logger/qslogging/QsLogDest.cpp \
           ../logger/qslogging/QsLogDestConsole.cpp \
           ../logger/qslogging/QsLogDestFile.cpp\
           ../logger/qslogging/QsLogDestFunctor.cpp \
           ../settings/CGlobalSettingController.cpp \
           ../workflow/workflow/aworkflow.cpp \
           ../workflow/workflow/cworkflowmanager.cpp \
           ../workflow/plugin/cpluginmanager.cpp \
           main.cpp

FORMS += ../gui/forms/CMainWindow.ui \
         ../gui/forms/CSettingsDialog.ui

DISTFILES += \
    io/data/testImage.png \
    io/data/testText.txt

RESOURCES += \
    test.qrc
