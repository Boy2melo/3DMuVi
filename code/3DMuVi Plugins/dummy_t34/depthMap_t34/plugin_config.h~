#ifndef PLUGIN_CONFIG_H
#define PLUGIN_CONFIG_H
#include "workflow/plugin/cpluginmanager.h"

//The name of your Plugin
#define _PLUGIN_NAME DummyDepthMap
//The type of your plugin
#define _PLUGIN_TYPE PT_DepthEstimator
//The Author of this plugin
#define _PLUGIN_AUTHOR Nathanael Schneider
//The Version of this plugin (uint)
#define _PLUGIN_VERSION 1
//The Date of this Plugin
#define _PLUGIN_DATE 2016-02-27



#define _CLASS_GEN(x) _concat1(_PLUGIN_NAME, x)
#define _concat1(x,y) _concat2(x, y)    //Double layer of indirection - otherwise _PLUGIN_NAME will not be resolved
#define _concat2(x,y) _concat3(x, y)
#define _concat3(x,y) x ## y
#define str(x) str1(x)
#define str1(x) str2(x)
#define str2(x) str3(x)
#define str3(x) #x

#endif // PLUGIN_CONFIG_H
