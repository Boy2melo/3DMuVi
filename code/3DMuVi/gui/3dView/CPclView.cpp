#ifdef PCL
#include <QVBoxLayout>

#include <vtkRenderWindow.h>

#include "CPclView.h"

#define CAMERA_SHAPE_N_VERTICES 5
#define CAMERA_SHAPE_N_LINES 8

CPclView::CPclView(QWidget *parent) : QWidget(parent)
{
  mpQVtkWidget.reset(new QVTKWidget(this));
  mpPclVisualizer.reset(new pcl::visualization::PCLVisualizer("", false));

  setLayout(new QVBoxLayout);
  layout()->addWidget(mpQVtkWidget.get());

  mpQVtkWidget->SetRenderWindow(mpPclVisualizer->getRenderWindow());
  mpPclVisualizer->setupInteractor(mpQVtkWidget->GetInteractor(), mpQVtkWidget->GetRenderWindow());
  mpQVtkWidget->update();
}

CPclView::~CPclView()
{

}

void CPclView::addPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
                             const QString& id)
{
  mpPclVisualizer->addPointCloud(pointCloud, id.toStdString());
}

void CPclView::addPolygonMesh(const pcl::PolygonMesh& mesh, const QString& id)
{
  mpPclVisualizer->addPolygonMesh(mesh, id.toStdString());
}

void CPclView::addTextureMesh(const pcl::TextureMesh &mesh, const QString& id)
{
  mpPclVisualizer->addTextureMesh(mesh, id.toStdString());
}

void CPclView::addCameraMesh(const pcl::PointXYZ& position, const Eigen::Quaternionf& rotation,
                             const QString& id)
{
  Eigen::Vector3f cameraBasePos(position.x, position.y, position.z);
  Eigen::Quaternionf quat = rotation.normalized();
  Eigen::Matrix3f rotationMatrix = quat.toRotationMatrix();
  Eigen::Vector3f cameraShapeVertices[CAMERA_SHAPE_N_VERTICES];
  pcl::PointXYZ pclCameraPoints[CAMERA_SHAPE_N_VERTICES];

  cameraShapeVertices[0] = Eigen::Vector3f( 0.f,  0.f, 0.f);
  cameraShapeVertices[1] = Eigen::Vector3f(-1.f, -1.f, 1.f);
  cameraShapeVertices[2] = Eigen::Vector3f(-1.f,  1.f, 1.f);
  cameraShapeVertices[3] = Eigen::Vector3f( 1.f, -1.f, 1.f);
  cameraShapeVertices[4] = Eigen::Vector3f( 1.f,  1.f, 1.f);

  for(int i = 0; i < CAMERA_SHAPE_N_VERTICES; i++)
  {
    cameraShapeVertices[i] = rotationMatrix * cameraShapeVertices[i];
    cameraShapeVertices[i] += cameraBasePos;

    pclCameraPoints[i] = pcl::PointXYZ(cameraShapeVertices[i](0), cameraShapeVertices[i](1),
                                       cameraShapeVertices[i](2));
  }

  mpPclVisualizer->addLine(pclCameraPoints[0], pclCameraPoints[1], id.toStdString() + "_0");
  mpPclVisualizer->addLine(pclCameraPoints[0], pclCameraPoints[2], id.toStdString() + "_1");
  mpPclVisualizer->addLine(pclCameraPoints[0], pclCameraPoints[3], id.toStdString() + "_2");
  mpPclVisualizer->addLine(pclCameraPoints[0], pclCameraPoints[4], id.toStdString() + "_3");
  mpPclVisualizer->addLine(pclCameraPoints[1], pclCameraPoints[2], id.toStdString() + "_4");
  mpPclVisualizer->addLine(pclCameraPoints[1], pclCameraPoints[3], id.toStdString() + "_5");
  mpPclVisualizer->addLine(pclCameraPoints[2], pclCameraPoints[4], id.toStdString() + "_6");
  mpPclVisualizer->addLine(pclCameraPoints[3], pclCameraPoints[4], id.toStdString() + "_7");
}

void CPclView::removePointCloud(const QString& id)
{
  mpPclVisualizer->removePointCloud(id.toStdString());
}

void CPclView::removePolygonMesh(const QString& id)
{
  mpPclVisualizer->removePolygonMesh(id.toStdString());
}

void CPclView::removeTextureMesh(const QString& id)
{
  mpPclVisualizer->removeShape(id.toStdString());
}

void CPclView::removeCameraMesh(const QString& id)
{
  for(int i = 0; i < CAMERA_SHAPE_N_LINES; i++)
  {
    mpPclVisualizer->removeShape(id.toStdString() + std::to_string(i));
  }
}

void CPclView::updatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
                                const QString& id)
{
  mpPclVisualizer->updatePointCloud(pointCloud, id.toStdString());
}

void CPclView::updatePolygonMesh(const pcl::PolygonMesh &mesh, const QString& id)
{
  mpPclVisualizer->updatePolygonMesh(mesh, id.toStdString());
}

void CPclView::updateTextureMesh(const pcl::TextureMesh &mesh, const QString& id)
{
  removeTextureMesh(id);
  addTextureMesh(mesh, id);
}

void CPclView::updateCameraMesh(const pcl::PointXYZ& position,
                                const Eigen::Quaternionf& rotation, const QString& id)
{
  removeCameraMesh(id);
  addCameraMesh(position, rotation, id);
}

//Not implemented
/*
void CPclView::showCoordinateSystem(bool state)
{

}

void CPclView::setBackground(float red, float blue, float green)
{

}

void CPclView::centerOnPoint(const pcl::PointXYZ& point)
{

}
*/
#endif
