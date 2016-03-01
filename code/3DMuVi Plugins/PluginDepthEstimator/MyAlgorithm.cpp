#include "algorithm.h"
#include "workflow/workflow/datapackets/CDataDepth.h"
#include "workflow/workflow/datapackets/CDataPose.h"
#include "workflow/workflow/datapackets/SPose.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"


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
    QDir dir = QDir(params->value("GtSrcDir").toString());
    dir.setNameFilters(QStringList() << "*.xml");
    dir.setSorting(QDir::Name);
    QStringList fileList = dir.entryList();
    QStringList::iterator fileItr = fileList.begin();

    if(fileList.empty())
    {
      std::cout << "\033[33m"
                << "WARNING: No Files in given source directory!"
                << "Directory: " << params->value("GtSrcDir").toString().toStdString()
                << "\033[0m"
                << std::endl;
      return false;
    }

    return true;
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
    auto pInputImages = store->getData<CInputDataSet>();
    auto pPoses = store->getData<CDataPose>();

    auto depthMaps = new std::vector<std::tuple<uint32_t, QImage>>;
    int frameCounter = 0;

    // read data from file
    for(std::tuple<uint32_t, QImage, CImagePreviewItem> img : *pInputImages->getInputImages())
    {
      frameCounter++;
      mFileItr++;

      if(frameCounter == mSettings->value("NumOfFrames").toInt())
      {
        //--- read depthmap ---
        cv::Mat depthMap;
        cv::FileStorage fs = cv::FileStorage(mSettings->value("GtSrcDir").toString().toStdString()
                                             + "/" + (*mFileItr).toStdString(),
                                             cv::FileStorage::READ);
        fs[mSettings->value("XmlDataId").toString().toStdString()] >> depthMap;

        frameCounter = 0;

        depthMaps->push_back(std::tuple<uint32_t, QImage>(std::get<0>(img), mat2Qimage(depthMap)));
      }
    }

    CDataDepth* depthData = new CDataDepth;
    depthData->setDepthMap(std::shared_ptr<std::vector<std::tuple<uint32_t, QImage>>>(depthMaps));

    store->appendData<CDataDepth>(std::shared_ptr<CDataDepth>(depthData), true);
}
