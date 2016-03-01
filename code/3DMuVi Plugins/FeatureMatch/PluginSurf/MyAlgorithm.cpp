#include "algorithm.h"

#include <vector>

#include <QImage>
#include <QJsonObject>
#include <QStringList>

#include <opencv2/opencv.hpp>

#include "surfmatch-algorithm.h"

#include "workflow/workflow/ccontextdatastore.h"

//#include "workflow/workflow/datapackets/CDataInputImages.h"
#include "workflow/workflow/datapackets/CDataFeature.h"

using namespace std;
using namespace cv;

//----------------------------------------
// Adjust these functions to your needs
//----------------------------------------

void _CLASS_GEN(Algorithm)::OnInitialize(){
    mInputTypes.push_back(DT_INPUTIMAGES);
    mOutputTypes.push_back(DT_FEATURE_MATCH);
}

bool _CLASS_GEN(Algorithm)::ValidateParameters(const QJsonObject *params) const{
    // First level typechecks are already done, see plugin.cpp
    Q_UNUSED(params);
    return true;
}

// mock up:
using InputImages = std::vector<QImage>;
struct CDataInputImages
{
    InputImages getImages() { return std::vector<QImage>(); }
};

void _CLASS_GEN(Algorithm)::executeAlgorithm(CContextDataStore *store){
  // Step 1: extract data from dataStore
  auto pInputPacket = store->getData<CDataInputImages>();
  if (pInputPacket == nullptr) return;
  auto inputImages = pInputPacket->getImages();
  vector<Mat> imgs(inputImages.size());
  transform(inputImages.begin(), inputImages.end(), imgs.begin(), [](QImage qImg) {
    qImg = qImg.convertToFormat(QImage::Format_ARGB32);
    auto img = Mat(qImg.height(), qImg.width(), CV_8UC4, qImg.bits(), qImg.bytesPerLine());
    return img.clone();
  });

  // Step 2: extract parameters from mSettings
  auto upright = mSettings->value("Upright").toBool(false);
  auto octaves = mSettings->value("Octaves").toInt(4);
  auto intervals = mSettings->value("Levels").toInt(3);
  auto init_sample = mSettings->value("Init Samples").toInt(2);
  auto thres = mSettings->value("Threshold").toDouble(0.00001);

  // Step 3: run algorithm
  auto result = findSurfMatches(imgs, upright, octaves, intervals, init_sample, thres);

  // Step 4: append result data to dataStore
  auto resultPacket = new CDataFeature;
  resultPacket->setFeatureMatch(std::shared_ptr<FeatureMatch>(new FeatureMatch(result)));
  store->appendData(std::shared_ptr<CDataFeature>(resultPacket), true);
}
