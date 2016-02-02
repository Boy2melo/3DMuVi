#include <QtTest>

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
  CImageIo imageIo;
  QImage testImage;
  QUrl testImagePath;
  QUrl testImagePathToSave;
};

void CTestCImageIo::initTestCase()
{
    testImagePathToSave.setUrl("data/testSaveImage.png");
    testImagePath.setUrl("data/testImage.png");
    testImage.load(testImagePath.path());
}

void CTestCImageIo::save()
{
  imageIo.save(testImage,testImagePathToSave);
  QImage toCheckImage;
  toCheckImage.load(testImagePathToSave.path());
  QCOMPARE(toCheckImage, testImage);
}

void CTestCImageIo::load()
{
  QCOMPARE(imageIo.load(testImagePath.path()), testImage);
}

QTEST_MAIN(CTestCImageIo)
#include  "CTestCImageIo.moc"
