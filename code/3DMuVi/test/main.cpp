#include <QTest>

#include "gui/CTestCLogWidget.h"
#include "io/CTestCImageIo.h"
#include "io/CTestCTextIo.h"

int main(int argc, char* argv[])
{
  CTestCLogWidget logWidget;
  CTestCImageIo imageIo;
  CTestCTextIo textIo;

  QApplication app(argc, argv);

  QTest::qExec(&logWidget, argc, argv);
  QTest::qExec(&imageIo, argc, argv);
  QTest::qExec(&textIo, argc, argv);

  return 0;
}
