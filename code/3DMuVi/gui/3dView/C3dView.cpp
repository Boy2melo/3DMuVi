#ifdef PCL
#include "C3dView.h"

C3dView::C3dView(QWidget* parent) : CPclView(parent)
{

}

void C3dView::setModelTypeSelector(QComboBox* modelTypeComboBox)
{
  modelTypeComboBox->addItem("Point Cloud", QVariant(POINT_CLOUD));
  modelTypeComboBox->addItem("Mesh", QVariant(POLYGON_MESH));
  modelTypeComboBox->addItem("Textured", QVariant(TEXTURE_MESH));

  connect(modelTypeComboBox, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
          this, &C3dView::onCurrentIndexChangedModelType);
}

void C3dView::activate()
{
  std::vector<uint32_t> imageIds;

  //TODO: Is the camera id the same as the image id? Why different types?
  for(uint64_t id : mPoseIds)
  {
    imageIds.push_back(id);
  }

  emit relevantImagesChanged(imageIds);
}

void C3dView::applyData(CDataFusion const* packet)
{
  mPointCloud = packet->getPointCloud();
  mPolygonMesh = packet->getPolygonMesh();
  mTextureMesh = packet->getTextureMesh();

  updateView();
}

void C3dView::applyData(CDataPose const* packet)
{
  std::vector<SPose> poses = *packet->getPose();

  for(uint64_t id : mPoseIds)
  {
    removeCameraMesh(QString("camera") + QString::number(id));
    mPoseIds.clear();
  }

  for(SPose p : poses)
  {
    pcl::PointXYZ position(p.translation.x(), p.translation.y(), p.translation.z());
    Eigen::Quaternionf rotation(p.orientation.scalar(), p.orientation.x(), p.orientation.y(),
                       p.orientation.z());

    addCameraMesh(position, rotation, QString("camera") + QString::number(p.cameraId));
    mPoseIds.push_back(p.cameraId);
  }
}

void C3dView::onCurrentIndexChangedModelType(int index)
{
  QComboBox* comboBox = qobject_cast<QComboBox*>(sender());

  if(comboBox)
  {
    int modelType = comboBox->itemData(index).toInt();

    if(modelType == POINT_CLOUD || modelType == POLYGON_MESH || modelType == TEXTURE_MESH)
    {
      mSelectedModelType = static_cast<E3dModelType>(modelType);
      updateView();
    }
  }
}

void C3dView::updateView()
{
  if(mPointCloudAdded)
  {
    removePointCloud();
    mPointCloudAdded = false;
  }

  if(mPolygonMeshAdded)
  {
    removePolygonMesh();
    mPolygonMeshAdded = false;
  }

  if(mTextureMeshAdded)
  {
    removeTextureMesh();
    mTextureMeshAdded = false;
  }

  switch(mSelectedModelType)
  {
    case POINT_CLOUD:
    {
      if(mPointCloud)
      {
        addPointCloud(mPointCloud);
        mPointCloudAdded = true;
      }
    }
    break;
    case POLYGON_MESH:
    {
      if(mPolygonMesh)
      {
        addPolygonMesh(*mPolygonMesh);
        mPolygonMeshAdded = true;
      }
    }
    break;
    case TEXTURE_MESH:
    {
      if(mTextureMesh)
      {
        addTextureMesh(*mTextureMesh);
        mTextureMeshAdded = true;
      }
    }
    break;
    default:
    break;
  }
}
#endif
