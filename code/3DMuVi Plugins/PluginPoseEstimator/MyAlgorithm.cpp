#include "cpluginposeestimator.h"
#include "workflow/workflow/datapackets/CDataFeature.h"
#include "workflow/workflow/datapackets/CDataPose.h"
#include "workflow/workflow/datapackets/SPose.h"
#include <QFile>
#include <cmath>
#include <iostream>
#define _USE_MATH_DEFINES

//----------------------------------------
// Adjust these functions to your needs
//----------------------------------------

void CPluginPoseEstimator::setImages(std::shared_ptr<CInputDataSet> images)
{
  mImages = images;
}

void CPluginPoseEstimator::setFeatureMatches(std::shared_ptr<CDataFeature> featureMatches)
{
  mFeatures = featureMatches;
}

std::shared_ptr<CDataPose> CPluginPoseEstimator::getPoses()
{
  return mPoses;
}

bool CPluginPoseEstimator::validateParameters(const QJsonObject params) const{
    if(!AAlgorithmConfig::validateParameters(params))
    {
      return false;
    }

    // check if file exists
    QFile gtSrcFile(params.value("GtSrcFile").toString());
    if (!gtSrcFile.exists()) return false;

    // check if delimiter is at least one character
    if (params.value("Delimiter").toString().length() == 0) return false;

    return true;
}

void CPluginPoseEstimator::executeAlgorithm(){

    //--- need to be done in order to use '.' as decimal seperator in the conversion to string ---
    std::setlocale(LC_ALL, "C");

    auto deg2Rad = [](float val){return (val / 180.f) * M_PI; };

    mPoseFileStream.close();
    openFileStream();

    // get input files
    auto pInputImages = mImages;
    auto pFeatureMatches = mFeatures;

    auto poses = new std::vector<SPose>;

    // read data from file
    for(std::tuple<uint32_t, QImage, CImagePreviewItem> img : *pInputImages->getInputImages())
    {
      //--- if new line is read, parse pose. otherwise return. ---
        std::string line;
        std::string delimiter = mSettings.value("Delimiter").toString().toStdString();
//        std::string delimiter = ",";
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



          newPose.cameraId = std::get<0>(img);
          newPose.focalLength = std::vector<float>(2, 615.f);
          newPose.principalPoint = QVector2D(320, 240);
          newPose.translation = QVector3D(poseVals[0], poseVals[1], poseVals[2]);
          newPose.eulerAngles = QVector3D(deg2Rad(poseVals[3]), deg2Rad(poseVals[4]), deg2Rad(poseVals[5]));

          double qW, qX, qY, qZ;
          cvtEulerToQuaternion(newPose.eulerAngles.x(), newPose.eulerAngles.y(),
                               newPose.eulerAngles.z(), qW, qX, qY, qZ);
          newPose.orientation.setScalar(qW);
          newPose.orientation.setX(qX);
          newPose.orientation.setY(qY);
          newPose.orientation.setZ(qZ);

          poses->push_back(newPose);
       }
    }

    auto poseData = new CDataPose;
    poseData->setPose(std::shared_ptr<std::vector<SPose>>(poses));

    mPoses = std::shared_ptr<CDataPose>(poseData);
}

void CPluginPoseEstimator::openFileStream()
{
  //--- open file stream ---
  mPoseFileStream.open(mSettings.value("GtSrcFile").toString().toStdString(), std::ios_base::in);
//  mPoseFileStream.open("/media/rufboi/DATA/Testdaten/3dmuvi/camera_track.txt", std::ios_base::in);
  if(!mPoseFileStream.is_open())
  {
    std::cout << "\033[33m"
              << "WARNING: Pose groundtruth file did not open. Check path!"
              << "File: " << mSettings.value("GtSrcFile").toString().toStdString()
              << "\033[0m"
              << std::endl;
  }
}

void CPluginPoseEstimator::cvtEulerToQuaternion(double const iAngleX, double const iAngleY,
                                                double const iAngleZ,
                                                double & oQW, double& oQX, double& oQY, double& oQZ)
{
  const double cosX = std::cos(iAngleX / 2.f);
  const double cosY = std::cos(iAngleY / 2.f);
  const double cosZ = std::cos(iAngleZ / 2.f);

  const double sinX = std::sin(iAngleX / 2.f);
  const double sinY = std::sin(iAngleY / 2.f);
  const double sinZ = std::sin(iAngleZ / 2.f);


  oQW = cosZ * cosY * cosX + sinZ * sinY * sinX;
  oQX = cosZ * cosY * sinX - sinZ * sinY * cosX;
  oQY = cosZ * sinY * cosX + sinZ * cosY * sinX;
  oQZ = sinZ * cosY * cosX - cosZ * sinY * sinX;

  // compute norm
  double norm = std::pow(oQW, 2) + std::pow(oQX, 2) + std::pow(oQY, 2) + std::pow(oQZ, 2);

  // normalize quaternion
  oQX /= norm;
  oQY /= norm;
  oQZ /= norm;
  oQW /= norm;
}
