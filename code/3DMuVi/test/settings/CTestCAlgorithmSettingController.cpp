#include <QtTest>

#include "CTestCAlgorithmSettingController.h"
void CTestCAlgorithmSettingController::initTestCase()
{
    testurl = QUrl(":/data/settings");
    controller = new CAlgorithmSettingController(testurl);
    testdata = QJsonObject();
    testdata.insert("test", QJsonValue("deadbeef"));
}

void CTestCAlgorithmSettingController::testrequestJson()
{
    QUrl directory;
    directory = QUrl(QUrl::fromLocalFile(QFileInfo("test.json").absoluteFilePath()));
    controller->requestQJson(directory);
    QJsonObject* temp = controller->getSetting("test.json");
    QCOMPARE(temp->value("test"), testdata.value("test"));
    delete temp;
}

void CTestCAlgorithmSettingController::testsaveJson()
{
    QUrl directory;
    directory.setPassword("atest");
    controller->saveQJson(testdata, directory);
    controller->requestQJson(directory);
    QJsonObject* temp = controller->getSetting("test.json");
    QCOMPARE(temp->value("test"), testdata.value("test"));
    delete temp;
}

void CTestCAlgorithmSettingController::cleanUpTestCase()
{
    delete controller;
}
