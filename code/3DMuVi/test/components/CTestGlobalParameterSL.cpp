#include "CTestGlobalParameterSL.h"
#include <settings/CGlobalSettingController.h>
void CTestGlobalParameterSL::test()
{
CGlobalSettingController cgsc;
QString test1 = "test";
QString test2 = "2";
QString test3 = "0";
QString test4 = "1";

cgsc.setSetting("resultDirectory",test1);
cgsc.setSetting("minLogLevel",test2);
cgsc.setSetting("logWindowEnabled",test3);
cgsc.setSetting("logDataEnabled",test4);
QUrl dir(":/results");
cgsc.exportTo(dir);
cgsc.resetToDefault();
cgsc.import(dir,"globalconfig");

QCOMPARE(cgsc.getSetting("resultDirectory"),test1);
QCOMPARE(cgsc.getSetting("minLogLevel"),test2);
QCOMPARE(cgsc.getSetting("logWindowEnabled"),test3);
QCOMPARE(cgsc.getSetting("logDataEnabled"),test4);
}
