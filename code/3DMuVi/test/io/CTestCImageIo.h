#include <io/CImageIo.h>

/*!
\brief Test for CImageIo.
\author Laurenz Thiel

This class is a unit test for CImageIo.
*/
class CTestCImageIo : public QObject
{
  Q_OBJECT

private slots:
  void initTestCase();

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
  CImageIo imageIo;
  QImage testImage;
  QUrl testImagePath;
  QUrl testImagePathToSave;
};
