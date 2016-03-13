#include <QtTest>

#include "CTestCGlobalSettingController.h"
void CTestCGlobalSettingController::initTestCase()
{
    controller = CGlobalSettingController();
}

void CTestCGlobalSettingController::testgetSetting()
{
    controller.resetToDefault();
    QCOMPARE(controller.getSetting("resultDirectory"), QString("none"));
}

void CTestCGlobalSettingController::testsetSetting()
{
    controller.setSetting("minLogLevel", "5");
    controller.setSetting("resultDirectory", "Hallo");
    QCOMPARE(controller.getSetting("minLogLevel"), QString("5"));
    controller.resetToDefault();
    QCOMPARE(controller.getSetting("resultDirectory"), QString("none"));
}

void CTestCGlobalSettingController::testexport()
{
    QUrl url = QUrl(":/data");
    controller.setSetting("resultDirectory", "Hallo");
    controller.exportTo(url);
    controller.resetToDefault();
    controller.import(url, QString("globalconfig"));
    QCOMPARE(controller.getSetting("resultDirectory"), QString("Hallo"));
    controller.resetToDefault();
}
