#include <QtTest>

#include <gui/CLogWidget.h>

/*!
\brief Test for CLogWidget.
\author Stefan Wolf

This class is a unit test for CLogWidget.
*/
class CTestCLogWidget : public QObject
{
  Q_OBJECT

private slots:
  void initTestCase();

  /*!
  \brief Test for onNewLogMessage.

  This is a test for onNewLogMessage with valid input data.
   */
  void onNewLogMessage();

  /*!
  \brief Test for onStateChangedDebug.

  This is a test for onStateChangedDebug with valid input data.
   */
  void onStateChangedDebug();

  /*!
  \brief Test for onStateChangedInfo.

  This is a test for onStateChangedInfo with valid input data.
   */
  void onStateChangedInfo();

  /*!
  \brief Test for onStateChangedWarning.

  This is a test for onStateChangedWarning with valid input data.
   */
  void onStateChangedWarning();

  /*!
  \brief Test for onStateChangedError.

  This is a test for onStateChangedError with valid input data.
   */
  void onStateChangedError();

private:
  CLogWidget mWidget;
};

void CTestCLogWidget::initTestCase()
{
  mWidget.onNewLogMessage("Test message", "01.01.2016 00:30", "INFO");
  mWidget.onNewLogMessage("Another test message", "10.01.2016 12:30", "DEBUG");
  mWidget.onNewLogMessage("Third message", "15.01.2016 00:30", "WARNING");
  mWidget.onNewLogMessage("Last test message", "30.01.2016 12:30", "ERROR");
}

void CTestCLogWidget::onNewLogMessage()
{
  QCOMPARE(mWidget.toPlainText(), QString("01.01.2016 00:30: INFO: Test message\n" \
                                          "10.01.2016 12:30: DEBUG: Another test message\n" \
                                          "15.01.2016 00:30: WARNING: Third message\n" \
                                          "30.01.2016 12:30: ERROR: Last test message"));
}

void CTestCLogWidget::onStateChangedDebug()
{
  mWidget.onStateChangedDebug(Qt::Unchecked);

  QCOMPARE(mWidget.toPlainText(), QString("01.01.2016 00:30: INFO: Test message\n" \
                                          "15.01.2016 00:30: WARNING: Third message\n" \
                                          "30.01.2016 12:30: ERROR: Last test message"));
}

void CTestCLogWidget::onStateChangedInfo()
{
  mWidget.onStateChangedInfo(Qt::Unchecked);

  QCOMPARE(mWidget.toPlainText(), QString("10.01.2016 12:30: DEBUG: Another test message\n" \
                                          "15.01.2016 00:30: WARNING: Third message\n" \
                                          "30.01.2016 12:30: ERROR: Last test message"));
}

void CTestCLogWidget::onStateChangedWarning()
{
  mWidget.onStateChangedWarning(Qt::Unchecked);

  QCOMPARE(mWidget.toPlainText(), QString("01.01.2016 00:30: INFO: Test message\n" \
                                          "10.01.2016 12:30: DEBUG: Another test message\n" \
                                          "30.01.2016 12:30: ERROR: Last test message"));
}

void CTestCLogWidget::onStateChangedError()
{
  mWidget.onStateChangedError(Qt::Unchecked);

  QCOMPARE(mWidget.toPlainText(), QString("01.01.2016 00:30: INFO: Test message\n" \
                                          "10.01.2016 12:30: DEBUG: Another test message\n" \
                                          "15.01.2016 00:30: WARNING: Third message\n"));
}

QTEST_MAIN(CTestCLogWidget)
#include  "CTestCLogWidget.moc"
