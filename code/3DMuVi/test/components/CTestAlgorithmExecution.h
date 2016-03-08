#ifndef CTESTALGORITHMEXECUTION_H
#define CTESTALGORITHMEXECUTION_H


#include <QTest>

/*!
\brief Testing algorithm execution with dummyplugins which each wait 2 seconds.
\author Laurenz Thiel

This test is specified as user interface test T3.4 in the specification sheet.
*/
class CTestAlgorithmExecution : public QObject
{
  Q_OBJECT

private slots:
  /*!
  \brief Runs the test.
  */
  void test();

private:
};

#endif // CTESTALGORITHMEXECUTION_H
