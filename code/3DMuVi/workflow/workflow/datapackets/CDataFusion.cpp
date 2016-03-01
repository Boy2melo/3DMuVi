#ifdef PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>

#include "workflow/workflow/ccontextdatastore.h"
#include "io/CSFStreamProvider.h"

#include "CDataFusion.h"

CDataFusion::CDataFusion()
{

}

CDataFusion::~CDataFusion()
{
  if(mpStream)
  {
    delete mpStream;
    mpStream = nullptr;
  }
}

QString CDataFusion::getDataType() const
{
  return DT_FUSION;
}

AStreamProvider* CDataFusion::getStreamProvider()
{
  if(mpStream)
  {
    delete mpStream;
    mpStream = nullptr;
  }

  mpStream = new CSFStreamProvider();

  return mpStream;
}

void CDataFusion::serialize(AStreamProvider* stream)
{
  QDir dir = stream->getDestination();

  pcl::io::savePCDFile(dir.absoluteFilePath("point_cloud.pcd").toStdString(),
                       *mPointCloudData);
  pcl::io::savePLYFile(dir.absoluteFilePath("polygon_mesh.ply").toStdString(), *mPolygonMeshData);
  pcl::io::saveOBJFile(dir.absoluteFilePath("texture_mesh.obj").toStdString(), *mTextureMeshData);
}

//TODO: this should be reimplemented using the pcl load funtions
void CDataFusion::deserialize(AStreamProvider* stream)
{
  QDir dir = stream->getDestination();

  mPointCloudData = PointCloud::Ptr(new PointCloud);
  mPolygonMeshData = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh);
  mTextureMeshData = pcl::TextureMesh::Ptr(new pcl::TextureMesh);

  pcl::io::loadPCDFile(dir.absoluteFilePath("point_cloud.pcd").toStdString(), *mPointCloudData);
  pcl::io::loadPLYFile(dir.absoluteFilePath("polygon_mesh.ply").toStdString(), *mPolygonMeshData);
  pcl::io::loadOBJFile(dir.absoluteFilePath("texture_mesh.obj").toStdString(), *mTextureMeshData);
}

void CDataFusion::setPointCloud(PointCloud::Ptr cloud)
{
  mPointCloudData = cloud;
}

PointCloud::Ptr CDataFusion::getPointCloud() const
{
  return mPointCloudData;
}

void CDataFusion::setPolygonMesh(pcl::PolygonMesh::Ptr mesh)
{
  mPolygonMeshData = mesh;
}

pcl::PolygonMesh::Ptr CDataFusion::getPolygonMesh() const
{
  return mPolygonMeshData;
}

void CDataFusion::setTextureMesh(pcl::TextureMesh::Ptr mesh)
{
  mTextureMeshData = mesh;
}

pcl::TextureMesh::Ptr CDataFusion::getTextureMesh() const
{
  return mTextureMeshData;
}
#endif
