#include "algorithm.h"
#include "workflow/workflow/datapackets/CDataFeature.h"
#include "workflow/workflow/datapackets/CDataPose.h"
#include "workflow/workflow/datapackets/SPose.h"
#include <QFile>
#include <cmath>
#define _USE_MATH_DEFINES

//----------------------------------------
// Adjust these functions to your needs
//----------------------------------------

void _CLASS_GEN(Algorithm)::OnInitialize(){
    mInputTypes.push_back(DT_FEATURE_MATCH);
    mInputTypes.push_back(DT_INPUTIMAGES);

    mOutputTypes.push_back(DT_POSE);
}

bool _CLASS_GEN(Algorithm)::ValidateParameters(const QJsonObject *params) const{
    // First level typechecks are already done, see plugin.cpp

    // close file stream if still open
    //TODO: close not allowed in const method. Is this really necessary?
    //if(mPoseFileStream.is_open())
    //  mPoseFileStream.close();

    // check if file exists
    QFile gtSrcFile(params->value("GtSrcFile").toString());
    if (!gtSrcFile.exists()) return false;

    // check if delimiter is at least one character
    if (params->value("Delimiter").toString().length() == 0) return false;

    return true;
}

void _CLASS_GEN(Algorithm)::executeAlgorithm(CContextDataStore *store){

    auto deg2Rad = [](float val){return (val / 180.f) * M_PI; };

    if(!mPoseFileStream.is_open()) openFileStream();

    // get input files
    auto pInputImages = store->getData<CInputDataSet>();
    auto pFeatureMatches = store->getData<CDataFeature>();

    auto poses = new std::vector<SPose>;

    // read data from file
    for(std::tuple<uint32_t, QImage, CImagePreviewItem> img : *pInputImages->getInputImages())
    {
      //--- if new line is read, parse pose. otherwise return. ---
        std::string line;
        std::string delimiter = mSettings->value("Delimiter").toString().toStdString();
        if(std::getline(mPoseFileStream, line))
        {
          SPose newPose;

          //--- tmp variables for file parsing ---
          std::vector<double> poseVals;
          std::string poseStr;
          size_t delimiterPos = 0;

          //--- extract poses from line ---
          while ((delimiterPos = line.find(delimiter)) != std::string::npos) {

            poseStr = line.substr(0, delimiterPos);
            line.erase(0, delimiterPos + delimiter.length());

            poseVals.push_back(std::stod(poseStr));
          }

          //--- extract last value ---
          if(line.length() > 0)
          {
            poseVals.push_back(std::stod(line));
          }



          newPose.focalLength = std::vector<float>(2, 615.f);
          newPose.principalPoint = QVector2D(320, 240);
          newPose.translation = QVector3D(poseVals[0], poseVals[1], poseVals[2]);
          newPose.eulerAngles = QVector3D(deg2Rad(poseVals[3]), deg2Rad(poseVals[4]), deg2Rad(poseVals[5]));

          poses->push_back(newPose);
       }
    }

    auto poseData = new CDataPose;
    poseData->setPose(std::shared_ptr<std::vector<SPose>>(poses));

    store->appendData<CDataPose>(std::shared_ptr<CDataPose>(poseData), true);
}

void _CLASS_GEN(Algorithm)::openFileStream()
{
  //--- open file stream ---
  mPoseFileStream.open(mSettings->value("GtSrcFile").toString().toStdString(), std::ios_base::in);
  if(!mPoseFileStream.is_open())
  {
    std::cout << "\033[33m"
              << "WARNING: Pose groundtruth file did not open. Check path!"
              << "File: " << mSettings->value("GtSrcFile").toString().toStdString()
              << "\033[0m"
              << std::endl;
  }
}
