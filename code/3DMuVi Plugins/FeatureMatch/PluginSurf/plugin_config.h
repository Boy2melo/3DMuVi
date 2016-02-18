#ifndef PLUGIN_CONFIG_H
#define PLUGIN_CONFIG_H
#include "workflow/plugin/cpluginmanager.h"

//The name of your Plugin
#define PLUGIN_NAME OpenSurf
//The type of your plugin
#define PLUGIN_TYPE PT_FeatureMatcher
//The Author of this plugin
#define PLUGIN_AUTHOR Bastian Erdnuess
//The Version of this plugin (uint)
#define PLUGIN_VERSION 1
//The Date of this Plugin
#define PLUGIN_DATE 2016-02-18



#define CLASS_GEN(x) concat1(PLUGIN_NAME, x)
#define concat1(x,y) concat2(x, y)    //Double layer of indirection - otherwise _PLUGIN_NAME will not be resolved
#define concat2(x,y) x ## y

#endif // PLUGIN_CONFIG_H
