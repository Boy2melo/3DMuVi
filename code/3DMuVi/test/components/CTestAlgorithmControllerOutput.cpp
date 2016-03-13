#include "CTestAlgorithmControllerOutput.h"
#include <QDir>
#include <QStringList>
#include <QJsonObject>

void CTestAlgorithmControllerOutput::test()
{
    QDir workingDir = QDir(".");
    workingDir.mkdir("CTestResult");
    workingDir.cd("CTestResult");
    CAlgorithmSettingController* controller =
            new CAlgorithmSettingController(QUrl(workingDir.path()));
    bool settingsfound = false;
    QJsonObject data = QJsonObject();
    data.insert("test", QJsonValue("deadbeef"));
    data.insert("testint", QJsonValue(1337));
    data.insert("testbool", QJsonValue("false"));
    controller->setSetting(QString("settings"), data);
    controller->exportTo(QUrl(workingDir.path()));
    QStringList allEntries = workingDir.entryList(QDir::AllEntries);
    for(QString entry : allEntries){
        if(entry == QString("settings.json")){
            settingsfound = true;
        }
    }
    QVERIFY2(settingsfound, "could not find settings");
    controller->requestQJson(QUrl(workingDir.path()));
    QJsonObject* testdata = controller->getSetting(QString("settings"));
    QCOMPARE(testdata->value("test"), data.value("test"));
    QCOMPARE(testdata->value("testint"), data.value("testint"));
    QCOMPARE(testdata->value("testbool"), data.value("testbool"));
}
