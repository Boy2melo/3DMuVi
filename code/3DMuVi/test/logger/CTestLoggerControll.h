#ifndef CTESTLOGGERCONTROLL_H
#define CTESTLOGGERCONTROLL_H

/*!
\brief Test for CLoggerControll.
\author Tim Brodbeck

This class is a unit test for CLoggerControll.
*/
#include <QTest>
#include <logger/controll/CLogController.h>


class CTestLoggerControll: public QObject
{
  Q_OBJECT
private slots:
  void initTestCase();

  /*!
  \brief Test for getHistory.

  This is a test getting the valid History.
   */
  void testgetHistory();
  /*!
  \brief Test for manageNewLogMessage.

  This is a test for save 3 Log Messages with valid Data / false type and empty type
   */
  void testmanageNewLogMessage();

private:
  //CLogController& controll;
  //CLogHistory& h;
};

#endif // CTESTLOGGERCONTROLL_H
