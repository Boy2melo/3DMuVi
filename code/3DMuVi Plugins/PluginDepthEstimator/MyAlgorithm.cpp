#include "algorithm.h"
#include "workflow/workflow/datapackets/CDataDepth.h"
#include "workflow/workflow/datapackets/CDataPose.h"
#include "workflow/workflow/datapackets/SPose.h"
#include "opencv2/core.hpp"


//----------------------------------------
// Adjust these functions to your needs
//----------------------------------------

void _CLASS_GEN(Algorithm)::OnInitialize(){
    mInputTypes.push_back(DT_INPUTIMAGES);
    mInputTypes.push_back(DT_POSE);

    mOutputTypes.push_back(DT_DEPTH);
}

bool _CLASS_GEN(Algorithm)::ValidateParameters(const QJsonObject *params) const{
    // First level typechecks are already done, see plugin.cpp

    // check if directory exists
    QDir gtSrcDir = QDir(params->value("GtSrcDir").toString());
    if (!gtSrcDir.exists()) return false;

    // check that xml data id is not empty
    if (params->value("XmlDataId").toString().length() == 0) return false;

    // check that num of frames i larger than 1
    if (params->value("NumOfFrames").toInt() < 2) return false;

    //--- get file list from directory ---
    QDir dir = QDir(params->value("GtSrcDir"));
    dir.setNameFilters(QStringList() << "*.xml");
    dir.setSorting(QDir::Name);
    mFileList = dir.entryList();
    mFileItr = mFileList.begin();

    if(mFileList.empty())
    {
      std::cout << "\033[33m"
                << "WARNING: No Files in given source directory!"
                << "Directory: " << params->value("GtSrcDir")
                << "\033[0m"
                << std::endl;
      return false;
    }

    return true;
}

// mock up:
using InputImages = std::vector<QImage>;
struct CDataInputImages
{
    InputImages getImages() { return std::vector<QImage>(); }
};

template <typename T>
T * CContextDataStore::getData() {
  return nullptr;
}

void _CLASS_GEN(Algorithm)::executeAlgorithm(CContextDataStore *store){

    auto mat2Qimage = [](cv::Mat const& src){
        //TODO support conversion of single channel float.

       cv::Mat temp(src.cols,src.rows,src.type()); // make the same cv::Mat
       cvtColor(src, temp,CV_BGR2RGB); // cvtColor Makes a copt, that what i need
       QImage dest= QImage((uchar*) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
       return dest;
    };

    // get input files
    auto pInputImages = store->getData<CDataInputImages>();
    auto pPoses = store->getData<CDataPose>();

    std::vector<QImage> depthMaps;
    int frameCounter = 0;

    // read data from file
    for(QImage img : pInputImages->getImages())
    {
      frameCounter++;
      mFileItr++;

      if(frameCounter == mSettings->value("NumOfFrames"))
      {
        //--- read depthmap ---
        cv::Mat depthMap;
        cv::FileStorage fs = cv::FileStorage(mSettings->value("GtSrcDir") + "/"
                                             + (*mFileItr).toStdString(),
                                             cv::FileStorage::READ);
        fs[mSettings->value("XmlDataId").toString().toStdString()] >> depthMap;

        frameCounter = 0;

        depthMaps.push_back(mat2Qimage(depthMap));
      }
    }

    CDataDepth depthData;
    depthData.setDepthMap(depthMaps);

    store->appendData<CDataDepth>(depthData, true);
}
