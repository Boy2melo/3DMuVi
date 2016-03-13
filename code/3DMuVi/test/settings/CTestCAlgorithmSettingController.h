#include <settings/CAlgorithmSettingController.h>
class CTestCAlgorithmSettingController : public QObject
{
    Q_OBJECT
private slots:
  void initTestCase();
  void testrequestJson();
  void testsaveJson();
  void cleanUpTestCase();
private:
  CAlgorithmSettingController* controller;
  QUrl testurl;
  QJsonObject testdata;
};
