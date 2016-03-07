#-------------------------------------------------
#
# Project created by QtCreator 2016-03-07T18:17:31
#
#-------------------------------------------------

QT       += core gui widgets testlib

TARGET = PluginAlgorithmSettingsTest
TEMPLATE = lib
CONFIG += plugin c++11

INCLUDEPATH += ../../3DMuVi/

SOURCES += algorithm.cpp MyAlgorithm.cpp plugin.cpp

HEADERS += algorithm.h plugin.h plugin_config.h

DISTFILES += Plugin.json
