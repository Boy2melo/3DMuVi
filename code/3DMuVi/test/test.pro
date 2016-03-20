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

PCL{
  DEFINES += PCL

  CONFIG += link_pkgconfig
  PKGCONFIG += pcl_common-1.8 pcl_geometry-1.8 pcl_visualization-1.8

  INCLUDEPATH += /usr/local/include/vtk-7.0/

  LIBS += -lboost_system -lvtkGUISupportQt-7.0 -lvtkCommonCore-7.0 -lvtkRenderingCore-7.0
  LIBS += -lvtkCommonDataModel-7.0 -lvtkCommonMath-7.0
}

unix:QMAKE_LFLAGS += -rdynamic

INCLUDEPATH += ../

HEADERS += gui/CTestCLogWidget.h \
           gui/CTestCMainWindow.h \
           gui/CTestCSettingsDialog.h \
           #gui/CTestCImagePreviewWidget.h \
           gui/CTestCImageTab.h \
           io/CTestCImageIo.h \
           io/CTestCTextIo.h \
           io/CTestCInputDataSet.h \
           io/CTestCResultContext.h \
           logger/CTestLoggerHistory.h \
           logger/CTestLoggerControll.h \
           components/CTestAlgorithmSettings.h \
           components/CTestAlgorithmExecution.h \
           components/CTestResultDir.h \
           components/CTestLoggerOutput.h \
           components/CTestAlgorithmControllerOutput.h \
           components/CTestGlobalParameterSL.h \
           components/CTestAlgorithmSelection.h \
           components/CTestTabLog.h \
           settings/CTestCGlobalSettingController.h\
           settings/CTestCAlgorithmSettingController.h\
           workflow/CTestCContextDataStore.h \
           workflow/CTestCFeatureView.h \
           workflow/CTestCWorkflow.h \
           workflow/CTestCWorkflowmanager.h \
           workflow/plugin/CTestCPluginmanager.h \
           workflow/testplugin/algorithm.h \
           workflow/testplugin/plugin.h \
           ../gui/AlgorithmSettings/CAlgorithmSettingsSaveLoadWidget.h \
           ../gui/AlgorithmSettings/CAlgorithmSettingsView.h \
           ../gui/ImageViews/CDepthMapView.h \
           ../gui/ImageViews/CFeatureView.h \
           ../gui/ImageViews/CImageView.h \
           ../gui/ImageViews/CInputImageView.h \
           ../gui/CAlgorithmSelector.h \
           ../gui/CDatasetSelector.h \
           ../gui/CDatasetSelectorItem.h \
           ../gui/CDataViewTabContainer.h \
           ../gui/CImagePreviewItem.h \
           ../gui/CImagePreviewWidget.h \
           ../gui/CLogWidget.h \
           ../gui/CMainWindow.h \
           ../gui/CSettingsDialog.h \
           ../gui/CStepComboBox.h \
           ../io/CImageIo.h \
           ../io/CInputDataSet.h \
           ../io/CMFStreamProvider.h \
           ../io/CResultContext.h \
           ../io/CSFStreamProvider.h \
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
           ../settings/CAlgorithmSettingController.h \
           ../settings/CAlgorithmSettingsModel.h \
           ../settings/CQJsonModel.h \
           ../settings/CQJsonItem.h \
           ../workflow/workflow/datapackets/CDataDepth.h \
           ../workflow/workflow/datapackets/CDataFeature.h \
           ../workflow/workflow/datapackets/CDataPose.h \
           ../workflow/workflow/fourphase/cfourphaseworkflow.h \
           ../workflow/workflow/aworkflow.h \
           ../workflow/plugin/ialgorithm.h \
           ../workflow/workflow/cworkflowmanager.h \
           ../workflow/workflow/ccontextdatastore.h \
           ../workflow/plugin/cpluginmanager.h \
           ../workflow/workflow/idatapacket.h \
           ../workflow/workflow/idataview.h
		

SOURCES += gui/CTestCLogWidget.cpp \
           gui/CTestCMainWindow.cpp \
           gui/CTestCSettingsDialog.cpp \
           #gui/CTestCImagePreviewWidget.cpp \
           gui/CTestCImageTab.cpp \
           io/CTestCImageIo.cpp \
           io/CTestCTextIo.cpp \
           io/CTestCInputDataSet.cpp \
           io/CTestCResultContext.cpp \
           logger/CTestLoggerHistory.cpp \
           logger/CTestLoggerControll.cpp \
           components/CTestAlgorithmSettings.cpp \
           components/CTestAlgorithmExecution.cpp \
           components/CTestResultDir.cpp \
           components/CTestLoggerOutput.cpp \
           components/CTestAlgorithmControllerOutput.cpp \
           components/CTestGlobalParameterSL.cpp \
           components/CTestAlgorithmSelection.cpp \
           components/CTestTabLog.cpp \
           settings/CTestCGlobalSettingController.cpp\
           settings/CTestCAlgorithmSettingController.cpp\
           workflow/CTestCContextDataStore.cpp \
           workflow/CTestCFeatureView.cpp \
           workflow/CTestCWorkflow.cpp \
           workflow/CTestCWorkflowmanager.cpp \
           workflow/plugin/CTestCPluginmanager.cpp \
           workflow/testplugin/algorithm.cpp \
           workflow/testplugin/plugin.cpp \
           ../gui/AlgorithmSettings/CAlgorithmSettingsSaveLoadWidget.cpp \
           ../gui/AlgorithmSettings/CAlgorithmSettingsView.cpp \
           ../gui/ImageViews/CDepthMapView.cpp \
           ../gui/ImageViews/CFeatureView.cpp \
           ../gui/ImageViews/CImageView.cpp \
           ../gui/ImageViews/CInputImageView.cpp \
           ../gui/CAlgorithmSelector.cpp \
           ../gui/CDatasetSelector.cpp \
           ../gui/CDatasetSelectorItem.cpp \
           ../gui/CDataViewTabContainer.cpp \
           ../gui/CImagePreviewItem.cpp \
           ../gui/CImagePreviewWidget.cpp \
           ../gui/CLogWidget.cpp \
           ../gui/CMainWindow.cpp \
           ../gui/CSettingsDialog.cpp \
           ../gui/CStepComboBox.cpp \
           ../io/AStreamProvider.cpp \
           ../io/CImageIo.cpp \
           ../io/CInputDataSet.cpp \
           ../io/CMFStreamProvider.cpp \
           ../io/CResultContext.cpp \
           ../io/CSFStreamProvider.cpp \
           ../io/CTextIo.cpp \
           ../logger/controll/CLogController.cpp \
           ../logger/controll/CLogHistory.cpp \
           ../logger/qslogging/QsLog.cpp \
           ../logger/qslogging/QsLogDest.cpp \
           ../logger/qslogging/QsLogDestConsole.cpp \
           ../logger/qslogging/QsLogDestFile.cpp\
           ../logger/qslogging/QsLogDestFunctor.cpp \
           ../settings/CGlobalSettingController.cpp \
           ../settings/CAlgorithmSettingController.cpp \
           ../settings/CAlgorithmSettingsModel.cpp \
           ../settings/CQJsonModel.cpp \
           ../settings/CQJsonItem.cpp \
           ../workflow/workflow/datapackets/CDataDepth.cpp \
           ../workflow/workflow/datapackets/CDataFeature.cpp \
           ../workflow/workflow/datapackets/CDataPose.cpp \
           ../workflow/workflow/fourphase/cfourphaseworkflow.cpp \
           ../workflow/workflow/cworkflowmanager.cpp \
           ../workflow/workflow/aworkflow.cpp \
           ../workflow/workflow/ccontextdatastore.cpp \
           ../workflow/plugin/cpluginmanager.cpp \
           ../workflow/workflow/idatapacket.cpp \
           ../workflow/workflow/idataview.cpp \
           main.cpp

PCL{
  HEADERS += ../gui/3dView/C3dView.h \
      ../gui/3dView/CPclView.h \
      ../workflow/workflow/datapackets/CDataFusion.h

  SOURCES += ../gui/3dView/C3dView.cpp \
    ../gui/3dView/CPclView.cpp \
    ../workflow/workflow/datapackets/CDataFusion.cpp
}

FORMS += ../gui/forms/CMainWindow.ui \
         ../gui/forms/CSettingsDialog.ui \
         ../gui/forms/CDatasetSelector.ui

DISTFILES += \
    io/data/testImage.png \
    io/data/testText.txt

RESOURCES += \
    test.qrc

globalconfig_install.path = $$OUT_PWD
globalconfig_install.files = $$PWD/../settings/globalconfig.json $$PWD/../settings/defaultglobalconfig.json

INSTALLS += globalconfig_install
