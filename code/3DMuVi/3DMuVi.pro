#-------------------------------------------------
#
# Project created by QtCreator 2016-01-19T12:35:45
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = 3DMuVi
TEMPLATE = app


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
    workflow/plugin/talgorithm.cpp

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
    workflow/workflow/aworkflow.h

FORMS    += \
    gui/mainwindow.ui
