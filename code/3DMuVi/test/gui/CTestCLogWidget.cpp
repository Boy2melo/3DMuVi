#include <QtTest>

#include "CTestCLogWidget.h"

void CTestCLogWidget::initTestCase()
{
  CLogController& log = CLogController::instance();
  log.getHistory().clearHistory();
  log.manageNewLogMessage("Test message", "01.01.2016 00:30", "INFO");
  log.manageNewLogMessage("Another test message", "10.01.2016 12:30", "DEBUG");
  log.manageNewLogMessage("Third message", "15.01.2016 00:30", "WARNING");
  log.manageNewLogMessage("Last test message", "30.01.2016 12:30", "ERROR");
}

void CTestCLogWidget::init()
{
  mWidget = new CLogWidget;
  mWidget->onNewLogMessage("Test message", "01.01.2016 00:30", "INFO");
  mWidget->onNewLogMessage("Another test message", "10.01.2016 12:30", "DEBUG");
  mWidget->onNewLogMessage("Third message", "15.01.2016 00:30", "WARNING");
  mWidget->onNewLogMessage("Last test message", "30.01.2016 12:30", "ERROR");
}

void CTestCLogWidget::onNewLogMessage()
{
  QCOMPARE(mWidget->toPlainText(), QString("01.01.2016 00:30: INFO: Test message\n" \
                                           "10.01.2016 12:30: DEBUG: Another test message\n" \
                                           "15.01.2016 00:30: WARNING: Third message\n" \
                                           "30.01.2016 12:30: ERROR: Last test message"));
}

void CTestCLogWidget::onStateChangedDebug()
{
  mWidget->onStateChangedDebug(Qt::Unchecked);

  QCOMPARE(mWidget->toPlainText(), QString("01.01.2016 00:30: INFO: Test message\n" \
                                           "15.01.2016 00:30: WARNING: Third message\n" \
                                           "30.01.2016 12:30: ERROR: Last test message"));
}

void CTestCLogWidget::onStateChangedInfo()
{
  mWidget->onStateChangedInfo(Qt::Unchecked);

  QCOMPARE(mWidget->toPlainText(), QString("10.01.2016 12:30: DEBUG: Another test message\n" \
                                           "15.01.2016 00:30: WARNING: Third message\n" \
                                           "30.01.2016 12:30: ERROR: Last test message"));
}

void CTestCLogWidget::onStateChangedWarning()
{
  mWidget->onStateChangedWarning(Qt::Unchecked);

  QCOMPARE(mWidget->toPlainText(), QString("01.01.2016 00:30: INFO: Test message\n" \
                                           "10.01.2016 12:30: DEBUG: Another test message\n" \
                                           "30.01.2016 12:30: ERROR: Last test message"));
}

void CTestCLogWidget::onStateChangedError()
{
  mWidget->onStateChangedError(Qt::Unchecked);

  QCOMPARE(mWidget->toPlainText(), QString("01.01.2016 00:30: INFO: Test message\n" \
                                           "10.01.2016 12:30: DEBUG: Another test message\n" \
                                           "15.01.2016 00:30: WARNING: Third message"));
}

void CTestCLogWidget::cleanup()
{
  delete mWidget;
}

void CTestCLogWidget::cleanupTestCase()
{
  CLogController::instance().getHistory().clearHistory();
}
