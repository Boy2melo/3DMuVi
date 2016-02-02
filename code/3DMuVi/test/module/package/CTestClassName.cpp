#include <QtTest>

/*!
\brief Example for a test class.
\author Stefan Wolf

This class is an example how to write a unit test for a class and shouldn't be compiled or
executed. The tested class is called TestClassName and is in the package called package and
the module called module.
*/
class CTestClassName : public QObject
{
  Q_OBJECT

private slots:
  /*!
  \brief Test for method.

  This is a test for the method called method.
   */
  void method();
};

void CTestClassName::method()
{
  ClassName obj;
  QCOMPARE(obj.member(), REFERENCE_VALUE);
}

QTEST_MAIN(CTestClassName)
#include  "CTestClassName.moc"
