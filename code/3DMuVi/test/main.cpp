#include <QTest>

#include "gui/CTestCLogWidget.h"
#include "gui/CTestCMainWindow.h"
#include "gui/CTestCSettingsDialog.h"
#include "io/CTestCImageIo.h"
#include "io/CTestCTextIo.h"

int main(int argc, char* argv[])
{
  CTestCLogWidget logWidget;
  CTestCMainWindow mainWindow;
  CTestCSettingsDialog settingsDialog;
  CTestCImageIo imageIo;
  CTestCTextIo textIo;

  QApplication app(argc, argv);

  QTest::qExec(&logWidget, argc, argv);
  QTest::qExec(&mainWindow, argc, argv);
  QTest::qExec(&settingsDialog, argc, argv);
  QTest::qExec(&imageIo, argc, argv);
  QTest::qExec(&textIo, argc, argv);

  return 0;
}
