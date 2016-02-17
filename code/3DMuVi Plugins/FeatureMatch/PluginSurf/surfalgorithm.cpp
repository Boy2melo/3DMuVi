#include "surfalgorithm.h"

#include <QImage>
#include <opencv2/opencv.hpp>

#include "workflow/workflow/ccontextdatastore.h"

#include "surfmatch-algorithm.h"

using namespace std;
using namespace cv;

//----------------------------------------------
// Autofunctions, no adjustments needed
//----------------------------------------------

bool SurfAlgorithm::IsBusy() const{
    return mIsBusy;
}

void SurfAlgorithm::setLogger(CLogController *controller) {
    mLogger = controller;
}

void SurfAlgorithm::setParameters(QJsonObject *settings){
    mSettings = settings;
}

//----------------------------------------------------------
// ToDo-Functions
//----------------------------------------------------------
SurfAlgorithm::SurfAlgorithm() { }

QStringList SurfAlgorithm::getInputDataTypes() const {
    QStringList dataTypes;
    dataTypes.push_back(DT_INPUT_IMAGES);

    return dataTypes;
}

QStringList SurfAlgorithm::getOutputDataTypes() const {
    QStringList dataTypes;
    dataTypes.push_back(DT_FEATURE_MATCH);

    return dataTypes;
}

void SurfAlgorithm::run(CContextDataStore *dataStore, std::function<void (CContextDataStore *)> callback){
    if (! IsBusy() && dataStore && mSettings && mLogger) {
      mIsBusy = true;
      run(dataStore);

      // Needs to be called when all calculations are done
      callback(dataStore);
      mIsBusy = false;
    }
}

void SurfAlgorithm::run(CContextDataStore * dataStore)
{
  // Step 1: extract data from dataStore
  auto pInputPacket = dataStore->getData<CDataInputImages>();
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
  CDataFeatureMatch resultPacket;
  resultPacket.setFeatureMatch(move(result));
  // TODO: dataStore->putPacket(resultPacket);
}


