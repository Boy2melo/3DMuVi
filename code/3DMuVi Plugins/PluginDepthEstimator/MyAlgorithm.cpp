#include "cplugindepthestimator.h"
#include "workflow/workflow/datapackets/CDataDepth.h"
#include "workflow/workflow/datapackets/CDataPose.h"
#include "workflow/workflow/datapackets/SPose.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

#define MAX_VALUE_UCHAR 255

//----------------------------------------
// Adjust these functions to your needs
//----------------------------------------

void CPluginDepthEstimator::setImages(std::shared_ptr<CInputDataSet> images)
{
  mImages = images;
}

void CPluginDepthEstimator::setPoses(std::shared_ptr<CDataPose> poses)
{
  mPoses = poses;
}

std::shared_ptr<CDataDepth> CPluginDepthEstimator::getDepthMaps()
{
  return mDepthMaps;
}

bool CPluginDepthEstimator::validateParameters(const QJsonObject params) const{
    if(!AAlgorithmConfig::validateParameters(params))
    {
      return false;
    }

    // check if directory exists
    QDir gtSrcDir = QDir(params.value("GtSrcDir").toString());
    if (!gtSrcDir.exists()) return false;

    // check that xml data id is not empty
    if (params.value("XmlDataId").toString().length() == 0) return false;

    // check that num of frames i larger than 1
    if (params.value("NumOfFrames").toInt() < 2) return false;

    //--- get file list from directory ---
    QDir dir = QDir(params.value("GtSrcDir").toString());
    dir.setNameFilters(QStringList() << "*.xml");
    dir.setSorting(QDir::Name);
    QStringList fileList = dir.entryList();
    QStringList::iterator fileItr = fileList.begin();

    if(fileList.empty())
    {
      std::cout << "\033[33m"
                << "WARNING: No Files in given source directory!"
                << "Directory: " << params.value("GtSrcDir").toString().toStdString()
                << "\033[0m"
                << std::endl;
      return false;
    }

    return true;
}

void CPluginDepthEstimator::executeAlgorithm(){

    auto mat2Qimage = [](cv::Mat const& src){
       cv::Mat temp;
       double max;
       cv::minMaxLoc(src, nullptr, &max);
       src.convertTo(temp, CV_8UC1, 1.f / max * MAX_VALUE_UCHAR);
       QImage dest = QImage((uchar*) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_Grayscale8);
       return dest.copy();
    };

    // get input files
    auto pInputImages = mImages;
    auto pPoses = mPoses;

    auto depthMaps = new std::vector<std::tuple<uint32_t, QImage>>;
    int frameCounter = 0;

    QDir dir = QDir(mSettings.value("GtSrcDir").toString());
    dir.setNameFilters(QStringList() << "*.xml");
    dir.setSorting(QDir::Name);
    mFileList = dir.entryList();
    mFileItr = mFileList.begin();

    // read data from file
    for(std::tuple<uint32_t, QImage, CImagePreviewItem> img : *pInputImages->getInputImages())
    {
      frameCounter++;

      if(frameCounter == mSettings.value("NumOfFrames").toInt())
      {
        //--- read depthmap ---
        cv::Mat depthMap;
        cv::FileStorage fs = cv::FileStorage(mSettings.value("GtSrcDir").toString().toStdString()
                                             + "/" + (*mFileItr).toStdString(),
                                             cv::FileStorage::READ);
        fs[mSettings.value("XmlDataId").toString().toStdString()] >> depthMap;

        frameCounter = 0;

        depthMaps->push_back(std::tuple<uint32_t, QImage>(std::get<0>(img), mat2Qimage(depthMap)));
      }

      mFileItr++;
    }

    CDataDepth* depthData = new CDataDepth;
    depthData->setDepthMap(std::shared_ptr<std::vector<std::tuple<uint32_t, QImage>>>(depthMaps));

    mDepthMaps = std::shared_ptr<CDataDepth>(depthData);
}
