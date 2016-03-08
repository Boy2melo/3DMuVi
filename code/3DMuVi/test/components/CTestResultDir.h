#ifndef CTESTRESULTDIR_H
#define CTESTRESULTDIR_H



#include <QTest>

/*!
\brief Testing Result Context creating folder structure
\author Tim Brodbeck

This test is specified as component test T2.2 in the specification sheet.
*/
class CTestResultDir : public QObject
{
  Q_OBJECT

private slots:
  /*!
  \brief Runs the test.
  */
  void test();

private:
};


#endif // CTESTRESULTDIR_H
