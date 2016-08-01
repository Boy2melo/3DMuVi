#include "pluginsurf.h"

#include <vector>
#include <tuple>
#include <memory>

#include <QImage>
#include <QJsonObject>
#include <QStringList>

#include <opencv2/opencv.hpp>

#include "surfmatch-algorithm.h"

#include "io/CInputDataSet.h"
#include "workflow/workflow/datapackets/CDataFeature.h"

using namespace std;
using namespace cv;

//----------------------------------------
// Adjust these functions to your needs
//----------------------------------------

void PluginSurf::setImages(std::shared_ptr<CInputDataSet> images)
{
  mImages = images;
}

std::shared_ptr<CDataFeature> PluginSurf::getFeatureMatches()
{
  return mFeatures;
}

void PluginSurf::executeAlgorithm(){
  // Step 1: get input data
  std::shared_ptr<CInputDataSet> pInputPacket = mImages;

  if (pInputPacket == nullptr) return;

  using TupleType = std::tuple<uint32_t, QImage, CImagePreviewItem>;
  const vector<TupleType>* inputImages = pInputPacket->getInputImages();

  vector<Mat> imgs(inputImages->size());
  transform(inputImages->begin(), inputImages->end(), imgs.begin(), [](TupleType const & tup) {
    QImage qImg = get<1>(tup).convertToFormat(QImage::Format_ARGB32);
    auto img = Mat(qImg.height(), qImg.width(), CV_8UC4, qImg.bits(), qImg.bytesPerLine());
    return img.clone();
  });

  // Step 2: extract parameters from mSettings
  auto upright = mSettings.value("Upright").toBool(false);
  auto octaves = mSettings.value("Octaves").toInt(5);
  auto intervals = mSettings.value("Levels").toInt(4);
  auto init_sample = mSettings.value("Init Samples").toInt(2);
  auto thres = mSettings.value("Threshold").toDouble(0.0004);

  // Step 3: run algorithm
  shared_ptr<FeatureTable> result(new FeatureTable);
  *result = findSurfMatches(imgs, upright, octaves, intervals, init_sample, thres);

  // Step 4: store result
  auto resultPacket = std::make_shared<CDataFeature>();
  resultPacket->setFeatureMatch(result);
  mFeatures = resultPacket;
}
