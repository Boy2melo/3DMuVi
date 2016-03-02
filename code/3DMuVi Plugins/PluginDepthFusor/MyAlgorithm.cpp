#include "algorithm.h"
#include "workflow/workflow/datapackets/CDataFeature.h"
#include "workflow/workflow/datapackets/CDataPose.h"
#include "workflow/workflow/datapackets/SPose.h"
#include "workflow/workflow/datapackets/CDataFusion.h"
#include <QFile>
#include <cmath>
#define _USE_MATH_DEFINES

//----------------------------------------
// Adjust these functions to your needs
//----------------------------------------

void CLASS_GEN(Algorithm)::OnInitialize(){
    mInputTypes.push_back(DT_INPUTIMAGES);
    mInputTypes.push_back(DT_POSE);
    mInputTypes.push_back(DT_DEPTH);

    mOutputTypes.push_back(DT_FUSION);
}

bool CLASS_GEN(Algorithm)::ValidateParameters(const QJsonObject *params) const{
    // First level typechecks are already done, see plugin.cpp

    // check if files exists
    if (!QFile(params->value("PcSrcFile").toString()).exists()) return false;
    if (!QFile(params->value("MeshSrcFile").toString()).exists()) return false;
    if (!QFile(params->value("TexMeshSrcFile").toString()).exists()) return false;

    return true;
}

void CLASS_GEN(Algorithm)::executeAlgorithm(CContextDataStore *store){

    // get input files
    auto pInputImages = store->getData<CInputDataSet>();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud = loadPointCoudFromPcd(mSettings->value("PcSrcFile").toString().toStdString());
    pcl::PolygonMesh::Ptr pMesh = loadMeshFromPly(mSettings->value("MeshSrcFile").toString().toStdString());
    pcl::PolygonMesh::Ptr pTexMesh = loadMeshFromPly(mSettings->value("TexMeshSrcFile").toString().toStdString());

    auto fusionData = new CDataFusion;
    fusionData->setPointCloud(loadPointCoudFromPcd(mSettings->value("PcSrcFile").toString().toStdString()));
    fusionData->setPolygonMesh(loadMeshFromPly(mSettings->value("MeshSrcFile").toString().toStdString()));
//    fusionData->setTextureMesh(loadMeshFromPly(mSettings->value("TexMeshSrcFile").toString().toStdString()));

    store->appendData<CDataFusion>(std::shared_ptr<CDataFusion>(fusionData), true);
}

pcl::PolygonMesh::Ptr CLASS_GEN(Algorithm)::loadMeshFromPly(const std::string &iFilePath) {
  pcl::PolygonMesh::Ptr pPolyMesh = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh());

  if(pcl::io::loadPLYFile(iFilePath, *pPolyMesh) == -1)
    PCL_ERROR ("Couldn't read file %s \n", iFilePath.c_str());

  return pPolyMesh;
}

//TODO: Changed PointXYZRGBA to PointXYZRGB since 3D-MuVi only supports PointXYZRGB. Do we have to support more types?
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CLASS_GEN(Algorithm)::loadPointCoudFromPcd(const std::string &iFilePath)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(iFilePath, *pPointCloud) == -1)
    PCL_ERROR ("Couldn't read file %s \n", iFilePath.c_str());

  return pPointCloud;
}
