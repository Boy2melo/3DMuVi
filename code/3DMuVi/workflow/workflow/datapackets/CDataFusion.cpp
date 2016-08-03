#ifdef PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>

#include "workflow/workflow/ccontextdatastore.h"
#include "io/CSFStreamProvider.h"

#include "CDataFusion.h"

QString CDataFusion::getDataType() const
{
  return DT_FUSION;
}

std::unique_ptr<AStreamProvider> CDataFusion::getStreamProvider()
{
  return std::unique_ptr<AStreamProvider>(new CSFStreamProvider);
}

void CDataFusion::serialize(AStreamProvider* stream)
{
  QDir dir = stream->getDestination();

  if(mPointCloudData)
  {
    pcl::io::savePCDFile(dir.absoluteFilePath("point_cloud.pcd").toStdString(), *mPointCloudData);
  }
  if(mPolygonMeshData)
  {
    pcl::io::savePLYFile(dir.absoluteFilePath("polygon_mesh.ply").toStdString(), *mPolygonMeshData);
  }
  if(mTextureMeshData)
  {
    pcl::io::saveOBJFile(dir.absoluteFilePath("texture_mesh.obj").toStdString(), *mTextureMeshData);
  }
}

void CDataFusion::deserialize(AStreamProvider* stream)
{
  QDir dir = stream->getDestination();
  QString pointCloudFileName = dir.absoluteFilePath("point_cloud.pcd");
  QString polygonMeshFileName = dir.absoluteFilePath("polygon_mesh.ply");
  QString textureMeshFileName = dir.absoluteFilePath("texture_mesh.obj");

  if(QFile::exists(pointCloudFileName))
  {
    mPointCloudData = PointCloud::Ptr(new PointCloud);

    if(pcl::io::loadPCDFile(pointCloudFileName.toStdString(), *mPointCloudData) < 0)
    {
      mPointCloudData.reset();
    }
  }
  if(QFile::exists(polygonMeshFileName))
  {
    mPolygonMeshData = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh);

    if(pcl::io::loadPLYFile(polygonMeshFileName.toStdString(), *mPolygonMeshData) < 0)
    {
      mPolygonMeshData.reset();
    }
  }
  if(QFile::exists(textureMeshFileName))
  {
    mTextureMeshData = pcl::TextureMesh::Ptr(new pcl::TextureMesh);

    if(pcl::io::loadOBJFile(textureMeshFileName.toStdString(), *mTextureMeshData) < 0)
    {
      mTextureMeshData.reset();
    }
  }
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

void CDataFusion::applyToDataview(IDataView* dataView) const
{
  dataView->applyData(shared_from_this());
}
#endif
