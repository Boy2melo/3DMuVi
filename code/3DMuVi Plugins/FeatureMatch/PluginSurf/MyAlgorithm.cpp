#include "algorithm.h"

#include <vector>
#include <tuple>
#include <memory>

#include <QImage>
#include <QJsonObject>
#include <QStringList>

#include <opencv2/opencv.hpp>

#include "surfmatch-algorithm.h"

#include "workflow/workflow/ccontextdatastore.h"
#include "io/CInputDataSet.h"
#include "workflow/workflow/datapackets/CDataFeature.h"

using namespace std;
using namespace cv;

//----------------------------------------
// Adjust these functions to your needs
//----------------------------------------

void CLASS_GEN(Algorithm)::OnInitialize(){
    mInputTypes.push_back(DT_INPUTIMAGES);
    mOutputTypes.push_back(DT_FEATURE_MATCH);
}

bool CLASS_GEN(Algorithm)::ValidateParameters(const QJsonObject *params) const{
    // First level typechecks are already done, see plugin.cpp
    Q_UNUSED(params);
    return true;
}


void CLASS_GEN(Algorithm)::executeAlgorithm(CContextDataStore *store){
  // Step 1: extract data from dataStore
  std::shared_ptr<CInputDataSet> pInputPacket = store->getData<CInputDataSet>();

  if (pInputPacket == nullptr) return;

  using TupleType = std::tuple<uint32_t, QImage, CImagePreviewItem>;
  vector<TupleType>* inputImages = pInputPacket->getInputImages();

  vector<Mat> imgs(inputImages->size());
  transform(inputImages->begin(), inputImages->end(), imgs.begin(), [](TupleType const & tup) {
    QImage qImg = get<1>(tup).convertToFormat(QImage::Format_ARGB32);
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
  shared_ptr<FeatureTable> result(new FeatureTable);
  *result = findSurfMatches(imgs, upright, octaves, intervals, init_sample, thres);

  // Step 4: append result data to dataStore
  auto resultPacket = std::make_shared<CDataFeature>();
  resultPacket->setFeatureMatch(result);
  store->appendData(resultPacket, true);
}
