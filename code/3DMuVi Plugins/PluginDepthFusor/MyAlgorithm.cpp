#include "cplugindepthfusor.h"
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

void CPluginDepthFusor::setImages(std::shared_ptr<CInputDataSet> images)
{
  mImages = images;
}

void CPluginDepthFusor::setDepthMaps(std::shared_ptr<CDataDepth> depthMaps)
{
  mDepthMaps = depthMaps;
}

std::shared_ptr<CDataFusion> CPluginDepthFusor::getFusion()
{
  return mFusion;
}

bool CPluginDepthFusor::validateParameters(const QJsonObject params) const{
    if(!AAlgorithmConfig::validateParameters(params))
    {
        return false;
    }

    // check if files exists
    if (!QFile(params.value("PcSrcFile").toString()).exists()) return false;
    if (!QFile(params.value("MeshSrcFile").toString()).exists()) return false;
    if (!QFile(params.value("TexMeshSrcFile").toString()).exists()) return false;

    return true;
}

void CPluginDepthFusor::executeAlgorithm(){

    // get input files
    auto pInputImages = mImages;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud = loadPointCoudFromPcd(mSettings.value("PcSrcFile").toString().toStdString());
    pcl::PolygonMesh::Ptr pMesh = loadMeshFromPly(mSettings.value("MeshSrcFile").toString().toStdString());
    pcl::PolygonMesh::Ptr pTexMesh = loadMeshFromPly(mSettings.value("TexMeshSrcFile").toString().toStdString());

    auto fusionData = new CDataFusion;
    fusionData->setPointCloud(loadPointCoudFromPcd(mSettings.value("PcSrcFile").toString().toStdString()));
    fusionData->setPolygonMesh(loadMeshFromPly(mSettings.value("MeshSrcFile").toString().toStdString()));
//    fusionData->setTextureMesh(loadMeshFromPly(mSettings.value("TexMeshSrcFile").toString().toStdString()));

    mFusion = std::shared_ptr<CDataFusion>(fusionData);
}

pcl::PolygonMesh::Ptr CPluginDepthFusor::loadMeshFromPly(const std::string &iFilePath) {
  pcl::PolygonMesh::Ptr pPolyMesh = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh());

  if(pcl::io::loadPLYFile(iFilePath, *pPolyMesh) == -1)
    PCL_ERROR ("Couldn't read file %s \n", iFilePath.c_str());

  return pPolyMesh;
}

//TODO: Changed PointXYZRGBA to PointXYZRGB since 3D-MuVi only supports PointXYZRGB. Do we have to support more types?
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CPluginDepthFusor::loadPointCoudFromPcd(const std::string &iFilePath)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(iFilePath, *pPointCloud) == -1)
    PCL_ERROR ("Couldn't read file %s \n", iFilePath.c_str());

  return pPointCloud;
}
