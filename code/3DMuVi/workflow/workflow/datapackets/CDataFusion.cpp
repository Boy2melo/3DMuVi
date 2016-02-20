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
  }

  mpStream = new CSFStreamProvider();

  return mpStream;
}

//TODO: this should be reimplemented using the pcl save funtions
void CDataFusion::serialize(AStreamProvider* stream)
{
  QDataStream* dataStream = nullptr;

  stream->setFileName(getId());
  dataStream = stream->getNextStream();

  *dataStream << (int)mPointCloudData->size();

  for(pcl::PointXYZRGB p : *mPointCloudData)
  {
    dataStream->writeRawData((char*)&p, sizeof(p));
  }
}

//TODO: this should be reimplemented using the pcl load funtions
void CDataFusion::deserialize(AStreamProvider* stream)
{
  QDataStream* dataStream = stream->getNextStream();
  int size;

  *dataStream >> size;
  mPointCloudData->resize(size);

  for(int i = 0; i < size; i++)
  {
    pcl::PointXYZRGB point;

    for(unsigned int i = 0; i < sizeof(point); i++)
    {
      dataStream->readRawData((char*)&point, sizeof(point));
    }

    mPointCloudData->push_back(point);
  }
}

void CDataFusion::setPointClound(PointCloud::Ptr cloud)
{
  mPointCloudData = cloud;
}

PointCloud::Ptr CDataFusion::getPointCloud()
{
  return mPointCloudData;
}

void CDataFusion::setPolygonMesh(pcl::PolygonMesh::Ptr mesh)
{
  mPolygonMeshData = mesh;
}

pcl::PolygonMesh::Ptr CDataFusion::getPolygonMesh()
{
  return mPolygonMeshData;
}

void CDataFusion::setTextureMesh(pcl::TextureMesh::Ptr mesh)
{
  mTextureMeshData = mesh;
}

pcl::TextureMesh::Ptr CDataFusion::getTextureMesh()
{
  return mTextureMeshData;
}
