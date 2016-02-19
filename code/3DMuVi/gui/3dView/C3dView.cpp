#include "C3dView.h"

C3dView::C3dView(QWidget* parent) : CPclView(parent)
{

}

void C3dView::setModelTypeSelector(QComboBox* modelTypeComboBox)
{
  modelTypeComboBox->addItem("Point Cloud", QVariant(POINT_CLOUD));
  modelTypeComboBox->addItem("Mesh", QVariant(MESH));
  modelTypeComboBox->addItem("Textured", QVariant(TEXTURED));

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

void C3dView::applyData(CDataFusion* packet)
{
  if(mCloudAdded)
  {
    updatePointCloud(*packet->getPointCloud());
  }
  else
  {
    addPointCloud(*packet->getPointCloud());
    mCloudAdded = true;
  }
}

void C3dView::applyData(CDataPose* packet)
{
  std::vector<SPose> poses = packet->getPose();

  for(uint64_t id : mPoseIds)
  {
    removeCameraMesh("camera" + id);
    mPoseIds.clear();
  }

  for(SPose p : poses)
  {
    pcl::PointXYZ position(p.translation.x(), p.translation.y(), p.translation.z());
    Eigen::Quaternionf rotation(p.orientation.scalar(), p.orientation.x(), p.orientation.y(),
                       p.orientation.z());

    addCameraMesh(position, rotation, "camera" + p.cameraId);
    mPoseIds.push_back(p.cameraId);
  }
}

//TODO: only point cloud available
void C3dView::onCurrentIndexChangedModelType(int index)
{
}
