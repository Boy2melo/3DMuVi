#include <QtTest>

#include "CTestCImageIo.h"

void CTestCImageIo::initTestCase()
{
    testImagePathToSave.setUrl("testSaveImage.png");
    testImagePath.setUrl(":/data/io/data/testImage.png");
    QCOMPARE(testImage.load(testImagePath.path()),true);
}

void CTestCImageIo::save()
{
  imageIo.save(testImage,testImagePathToSave);
  QImage toCheckImage;
  toCheckImage.load(testImagePathToSave.path());
  QCOMPARE(toCheckImage, testImage);
  QFile::remove(testImagePathToSave.path());
}

void CTestCImageIo::load()
{
  QCOMPARE(imageIo.load(testImagePath.path()), testImage);
}
