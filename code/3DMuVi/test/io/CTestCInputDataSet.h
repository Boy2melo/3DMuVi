#include <io/CInputDataSet.h>

/*!
\brief Test for CInputDataSet.
\author Laurenz Thiel

This class is a unit test for CInputDataSet.
*/
class CTestCInputDataSet : public QObject
{
  Q_OBJECT

private slots:
  void initTestCase();

  /*!
  \brief Test for getInputImages.

  This is a test for load with valid input data and compare the Ids.
   */
  void getInputImagesIds();

  /*!
  \brief Test for getInputImages.

  This is a test for load with valid input data and compare the QImages.
   */
  void getInputImagesQImages();

  /*!
  \brief Test for getInputImages.

  This is a test for load with valid input data and compare the CImagePreviewItem.
   */
  void getInputImagesCImagePreviewItems();

private:
  QUrl pathToInputDataSet;
  QUrl pathToImage0;
  QUrl pathToImage1;
  QUrl pathToImage2;
  QImage testImage0;
  QImage testImage1;
  QImage testImage2;
  CInputDataSet inputDataSet;
  std::vector<std::tuple<uint32_t,QImage,CImagePreviewItem>>* data;
};
