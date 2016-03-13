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

#include "settings/CTestCAlgorithmSettingController.h"
#include "settings/CTestCGlobalSettingController.h"

#include "workflow/plugin/CTestCPluginmanager.h"
#include "workflow/CTestCWorkflowmanager.h"
#include "workflow/CTestCContextDataStore.h"
#include "workflow/CTestCWorkflow.h"
#include "gui/CTestCImageTab.h"

#include "components/CTestAlgorithmSettings.h"
#include "components/CTestAlgorithmExecution.h"
#include "components/CTestGlobalParameterSL.h"
#include "components/CTestResultDir.h"
#include "components/CTestLoggerOutput.h"
#include "components/CTestAlgorithmControllerOutput.h"


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

  CTestCAlgorithmSettingController algorithmcontroller;
  CTestCGlobalSettingController globalcontroller;

  CTestCPluginmanager pluginmanager;
  CTestCWorkflowmanager workflowmanager;
  CTestCContextDataStore datastore;
  CTestCWorkflow workflow;
  CTestCImageTab imageTab;
  CTestAlgorithmSettings algorithmSettings;
  CTestAlgorithmExecution algorithmExecution;
  CTestGlobalParameterSL globalParameterSL;
  CTestResultDir resultDir;
  CTestLoggerOutput logoutput;
  CTestAlgorithmControllerOutput algorithmoutput;

  QApplication app(argc, argv);

  int failCounter = 0;
  failCounter += QTest::qExec(&logWidget, argc, argv);
  failCounter += QTest::qExec(&mainWindow, argc, argv);
  failCounter += QTest::qExec(&settingsDialog, argc, argv);

  failCounter += QTest::qExec(&imageIo, argc, argv);
  failCounter += QTest::qExec(&textIo, argc, argv);
  failCounter += QTest::qExec(&inputDataSet, argc, argv);
  //failCounter += QTest::qExec(&resultContext, argc, argv); Fehler in Datapackets führt zu einem seg fault.

  failCounter += QTest::qExec(&logControll, argc, argv);
  failCounter += QTest::qExec(&logHistory, argc, argv);

  failCounter += QTest::qExec(&algorithmcontroller, argc, argv);
  failCounter += QTest::qExec(&globalcontroller, argc, argv);

  //failCounter += QTest::qExec(&datastore, argc, argv);
  //failCounter += QTest::qExec(&workflow, argc, argv);
  failCounter += QTest::qExec(&imageTab, argc, argv);
  failCounter += QTest::qExec(&pluginmanager, argc, argv);
  failCounter += QTest::qExec(&workflowmanager, argc, argv);
  failCounter += QTest::qExec(&datastore, argc, argv);
  failCounter += QTest::qExec(&workflow, argc, argv);

  failCounter += QTest::qExec(&algorithmSettings, argc, argv);
  failCounter += QTest::qExec(&algorithmExecution, argc, argv);
  failCounter += QTest::qExec(&globalParameterSL, argc, argv);
  //failCounter += QTest::qExec(&resultDir, argc, argv); Fehler in Datapackets führt zu einem seg fault.
  failCounter += QTest::qExec(&logoutput, argc, argv);
  failCounter += QTest::qExec(&algorithmoutput, argc, argv);

  std::cout << "Total number of fails: " << failCounter << "\n";
  return 0;  
}
