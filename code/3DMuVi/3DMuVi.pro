#-------------------------------------------------
#
# Project created by QtCreator 2016-01-19T12:35:45
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = 3DMuVi
TEMPLATE = app

CONFIG += c++11

SOURCES +=\
    gui/mainwindow.cpp \
    main.cpp \
    workflow/plugin/aplugin.cpp \
    workflow/workflow/acontextdatastore.cpp \
    workflow/workflow/idatapacket.cpp \
    workflow/workflow/tdatapacket.cpp \
    workflow/workflow/idataview.cpp \
    workflow/plugin/cpluginmanager.cpp \
    workflow/workflow/cworkflowmanager.cpp \
    workflow/workflow/aworkflow.cpp \
    workflow/plugin/talgorithm.cpp \
    gui/3dView/C3dView.cpp \
    gui/AlgorithmSettings/CAlgorithmSettingsSaveLoadWidget.cpp \
    gui/AlgorithmSettings/CAlgoritmSettingsView.cpp \
    gui/ImageViews/CDepthMapView.cpp \
    gui/ImageViews/CFeatureView.cpp \
    gui/ImageViews/CImageView.cpp \
    gui/ImageViews/CInputImageView.cpp \
    gui/CAlgorithmSelector.cpp \
    gui/CDatasetSelector.cpp \
    gui/CDataViewTabContainer.cpp \
    gui/CImagePreviewItem.cpp \
    gui/CImagePreviewWidget.cpp \
    gui/CLogWidget.cpp \
    gui/CMainWindow.cpp \
    gui/CSettingsDialog.cpp

HEADERS  += \
    workflow/plugin/idataaccess.h \
    gui/mainwindow.h \
    workflow/plugin/aplugin.h \
    workflow/plugin/ialgorithm.h \
    workflow/plugin/talgorithm.h \
    workflow/workflow/acontextdatastore.h \
    workflow/workflow/idatapacket.h \
    workflow/workflow/tdatapacket.h \
    workflow/workflow/idataview.h \
    workflow/plugin/cpluginmanager.h \
    workflow/workflow/cworkflowmanager.h \
    workflow/workflow/aworkflow.h \
    gui/3dView/C3dView.h \
    gui/AlgorithmSettings/CAlgorithmSettingsSaveLoadWidget.h \
    gui/AlgorithmSettings/CAlgoritmSettingsView.h \
    gui/ImageViews/CDepthMapView.h \
    gui/ImageViews/CFeatureView.h \
    gui/ImageViews/CImageView.h \
    gui/ImageViews/CInputImageView.h \
    gui/CAlgorithmSelector.h \
    gui/CDatasetSelector.h \
    gui/CDataViewTabContainer.h \
    gui/CImagePreviewItem.h \
    gui/CImagePreviewWidget.h \
    gui/CLogWidget.h \
    gui/CMainWindow.h \
    gui/CSettingsDialog.h \
    gui/IGuiDataView.h \
    workflow/plugin/ealgorithmtype.h

FORMS    += \
    gui/mainwindow.ui
