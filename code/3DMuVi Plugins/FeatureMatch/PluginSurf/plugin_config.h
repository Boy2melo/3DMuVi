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



#define _CLASS_GEN(x) _concat1(_PLUGIN_NAME, x)
#define _concat1(x,y) _concat2(x, y)    //Double layer of indirection - otherwise _PLUGIN_NAME will not be resolved
#define _concat2(x,y) _concat3(x, y)
#define _concat3(x,y) x ## y
#define str(x) str1(x)
#define str1(x) str2(x)
#define str2(x) str3(x)
#define str3(x) #x

#endif // PLUGIN_CONFIG_H
