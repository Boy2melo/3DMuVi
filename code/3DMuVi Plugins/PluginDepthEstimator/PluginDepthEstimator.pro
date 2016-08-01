#-------------------------------------------------
#
# Project created by QtCreator 2016-02-15T22:18:43
#
#-------------------------------------------------

QT       += core gui widgets

TARGET = PluginDepthEstimator
TEMPLATE = lib
CONFIG += plugin

#DESTDIR = $$[QT_INSTALL_PLUGINS]/generic
DESTDIR = ../../Win32/Debug/plugins

QMAKE_CXXFLAGS += -std=c++11
QMAKE_CXXFLAGS_RELEASE += -O3

# add suffix d to debug output
CONFIG(debug, debug|release) {
  win32: TARGET =  $$join(TARGET,,,d)
  unix: TARGET = $$join(TARGET,,,_debug)
}

SOURCES += \
    MyAlgorithm.cpp \
    cplugindepthestimator.cpp

INCLUDEPATH += ../../3DMuVi

DEFINES += BUILD_NAME=\\\"plugins/$${TARGET}\\\"

HEADERS += \
    ../../3DMuVi/workflow/plugin/ialgorithm.h \
    cplugindepthestimator.h

OTHER_FILES += \
    PluginDepthEstimator.json

unix {
    target.path = /usr/lib
    INSTALLS += target
}

#############
# OpenCV
unix {
  #INCLUDEPATH += /home/rufboi/3rd_party/OpenCV_3.1.0_gcc4.8/include/
  #DEPENDPATH += /home/rufboi/3rd_party/OpenCV_3.1.0_gcc4.8/include/

  #LIBS += -L/home/rufboi/3rd_party/OpenCV_3.1.0_gcc4.8/lib/
  LIBS += -lopencv_core -lopencv_imgproc
}
