#include <io/CTextIo.h>

/*!
\brief Test for CTextIo.
\author Laurenz Thiel

This class is a unit test for CTextIo.
*/
class CTestCTextIo : public QObject
{
  Q_OBJECT

private slots:
  void initTestCase();
  void cleanupTestCase();

  /*!
  \brief Test for save.

  This is a test for save with valid input data.
   */
  void save();

  /*!
  \brief Test for load.

  This is a test for load with valid input data.
   */
  void load();

private:
  CTextIo textIo;
  QString testText;
  QUrl testTextPath;
  QUrl testTextPathToSave;
};
