#ifndef CTESTLOGGERHISTORY_H
#define CTESTLOGGERHISTORY_H

/*!
\brief Test for CTestLoggerHistory.
\author Tim Brodbeck

This class is a unit test for CTestLoggerHistory.
*/
#include <QTest>
#include <logger/controll/CLogHistory.h>


class CTestLoggerHistory: public QObject
{
  Q_OBJECT
private slots:

  /*!
  \brief Test for getHistory.

  This is a test getting the valid History.
   */
  void testgetHistory();
  /*!
  \brief Test for addHistory.

  This is a test for valid adds to the History;
   */
  void testaddHistory();


private:

};

#endif // CTESTLOGGERHISTORY_H
