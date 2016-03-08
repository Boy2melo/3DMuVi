#include <QTest>

#include "gui/CTestCLogWidget.h"
#include "gui/CTestCMainWindow.h"
#include "gui/CTestCSettingsDialog.h"
#include "io/CTestCImageIo.h"
#include "io/CTestCTextIo.h"
#include "io/CTestCInputDataSet.h"
#include "io/CTestCResultContext.h"
#include "logger/CTestLoggerControll.h"
#include "logger/CTestLoggerHistory.h"
#include "components/CTestAlgorithmSettings.h"
#include "settings/CTestCAlgorithmSettingController.h"
#include "components/CTestAlgorithmExecution.h"
#include "settings/CTestCGlobalSettingController.h"


int main(int argc, char* argv[])
{
  CTestCLogWidget logWidget;
  CTestCMainWindow mainWindow;
  CTestCSettingsDialog settingsDialog;
  CTestCImageIo imageIo;
  CTestCTextIo textIo;
  CTestCInputDataSet inputDataSet;
  CTestCResultContext resultContext;
  CTestLoggerControll logControll;
  CTestLoggerHistory logHistory;
  CTestAlgorithmSettings algorithmSettings;
  CTestCAlgorithmSettingController algorithmcontroller;
  CTestAlgorithmExecution algorithmExecution;
  CTestCGlobalSettingController globalcontroller;

  QApplication app(argc, argv);

  int failCounter = 0;
  failCounter += QTest::qExec(&logWidget, argc, argv);
  failCounter += QTest::qExec(&mainWindow, argc, argv);
  failCounter += QTest::qExec(&settingsDialog, argc, argv);
  failCounter += QTest::qExec(&imageIo, argc, argv);
  failCounter += QTest::qExec(&textIo, argc, argv);
  failCounter += QTest::qExec(&inputDataSet, argc, argv);
  failCounter += QTest::qExec(&resultContext, argc, argv);
  failCounter += QTest::qExec(&logControll, argc, argv);
  failCounter += QTest::qExec(&logHistory, argc, argv);
  failCounter += QTest::qExec(&algorithmSettings, argc, argv);
  failCounter += QTest::qExec(&algorithmcontroller, argc, argv);
  failCounter += QTest::qExec(&algorithmExecution, argc, argv);
  failCounter += QTest::qExec(&globalcontroller, argc, argv);
  std::cout << "Total number of fails: " << failCounter << "\n";

  return 0;
}
