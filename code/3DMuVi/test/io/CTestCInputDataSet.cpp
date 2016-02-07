#include <QtTest>

#include "CTestCInputDataSet.h"

void CTestCInputDataSet::initTestCase()
{
    pathToInputDataSet.setUrl(":/data/io/data/testInputDataSet/");
    pathToImage0.setUrl(":/data/io/data/testInputDataSet/0.png");
    pathToImage1.setUrl(":/data/io/data/testInputDataSet/1.png");
    pathToImage2.setUrl(":/data/io/data/testInputDataSet/2.png");
    QCOMPARE(testImage0.load(pathToImage0.path()),true);
    QCOMPARE(testImage1.load(pathToImage1.path()),true);
    QCOMPARE(testImage2.load(pathToImage2.path()),true);
}

void CTestCInputDataSet::getInputImagesIds()
{
    CInputDataSet inputDataSet(pathToInputDataSet);
    data = inputDataSet.getInputImages();
    QCOMPARE(std::get<0>((*data)[0]), (uint32_t)0);
    QCOMPARE(std::get<0>((*data)[1]), (uint32_t)1);
    QCOMPARE(std::get<0>((*data)[2]), (uint32_t)2);
}

void CTestCInputDataSet::getInputImagesQImages()
{
    CInputDataSet inputDataSet(pathToInputDataSet);
    data = inputDataSet.getInputImages();
    QCOMPARE(std::get<1>((*data)[0]), testImage0);
    QCOMPARE(std::get<1>((*data)[1]), testImage1);
    QCOMPARE(std::get<1>((*data)[2]), testImage2);
}

void CTestCInputDataSet::getInputImagesCImagePreviewItems()
{
    CInputDataSet inputDataSet(pathToInputDataSet);
    data = inputDataSet.getInputImages();

    CImagePreviewItem imagePrewviewItem0 = std::get<2>((*data)[0]);
    CImagePreviewItem imagePrewviewItem1 = std::get<2>((*data)[1]);
    CImagePreviewItem imagePrewviewItem2 = std::get<2>((*data)[2]);

    //Compare the QIcons
    QCOMPARE(imagePrewviewItem0.icon().pixmap(500,500).toImage(), testImage0);
    QCOMPARE(imagePrewviewItem1.icon().pixmap(500,500).toImage(), testImage1);
    QCOMPARE(imagePrewviewItem2.icon().pixmap(500,500).toImage(), testImage2);

    //Compare the imagePrewviewItemIds
    QCOMPARE(imagePrewviewItem0.getImageId(), (uint32_t)0);
    QCOMPARE(imagePrewviewItem1.getImageId(), (uint32_t)1);
    QCOMPARE(imagePrewviewItem2.getImageId(), (uint32_t)2);

    //Compage the file namens
    QCOMPARE(imagePrewviewItem0.text(), QString("0.png"));
    QCOMPARE(imagePrewviewItem1.text(), QString("1.png"));
    QCOMPARE(imagePrewviewItem2.text(), QString("2.png"));
}
