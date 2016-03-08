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
    QCOMPARE(controller.getSetting("minLogLevel"), QString("5"));
    controller.resetToDefault();
}

void CTestCGlobalSettingController::testexport()
{
    QUrl url = QUrl(":/");
    controller.setSetting("resultDirectory", "Hallo");
    controller.exportTo(url);
    controller.resetToDefault();
    controller.resetToDefault();
    QCOMPARE(controller.getSetting("resultDirectory"), QString("none"));
}
