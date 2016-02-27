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
    mInputTypes.push_back(DT_INPUTIMAGES);
    mInputTypes.push_back(DT_POSE);
    mInputTypes.push_back(DT_DEPTH);

    mOutputTypes.push_back(DT_FUSION);
}

bool _CLASS_GEN(Algorithm)::ValidateParameters(const QJsonObject *params) const{
    // First level typechecks are already done, see plugin.cpp

    // check if files exists
    if (!QFile(parameterList->value("PcSrcFile").toString()).exists()) return false;
    if (!QFile(parameterList->value("MeshSrcFile").toString()).exists()) return false;
    if (!QFile(parameterList->value("TexMeshSrcFile").toString()).exists()) return false;

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

    // get input files
    auto pInputImages = store->getData<CDataInputImages>();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pPointCloud = loadPointCoudFromPcd(mSettings->value("PcSrcFile").toString().toStdString());
    pcl::PolygonMesh::Ptr pMesh = loadMeshFromPly(mSettings->value("MeshSrcFile").toString().toStdString());
    pcl::PolygonMesh::Ptr pTexMesh = loadMeshFromPly(mSettings->value("TexMeshSrcFile").toString().toStdString());

    CDataFusion fusionData;
    fusionData.setPointCloud(loadPointCoudFromPcd(mSettings->value("PcSrcFile").toString().toStdString()));
    fusionData.setPolygonMesh(loadMeshFromPly(mSettings->value("MeshSrcFile").toString().toStdString()));
    fusionData.setTextureMesh(loadMeshFromPly(mSettings->value("TexMeshSrcFile").toString().toStdString()));

    store->appendData<CDataFusion>(fusionData, true);
}

pcl::PolygonMesh::Ptr _CLASS_GEN(Algorithm)::loadMeshFromPly(const string &iFilePath) {
  pcl::PolygonMesh::Ptr pPolyMesh = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh());

  if(pcl::io::loadPLYFile(iFilePath, *pPolyMesh) == -1)
    PCL_ERROR ("Couldn't read file %s \n", iFilePath.c_str());

  return pPolyMesh;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _CLASS_GEN(Algorithm)::loadPointCoudFromPcd(const string &iFilePath)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pPointCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

  if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(iFilePath, *pPointCloud) == -1)
    PCL_ERROR ("Couldn't read file %s \n", iFilePath.c_str());

  return pPointCloud;
}
