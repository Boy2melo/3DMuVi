#ifdef PCL
#include <QVBoxLayout>

#include <vtkRenderWindow.h>

#include "CPclView.h"

#define CAMERA_SHAPE_N_VERTICES 5

CPclView::CPclView(QWidget *parent) : QWidget(parent)
{
  mpQVtkWidget.reset(new QVTKWidget(this));
  mpPclVisualizer.reset(new pcl::visualization::PCLVisualizer("", false));

  setLayout(new QVBoxLayout);
  layout()->addWidget(mpQVtkWidget.get());

  mpQVtkWidget->SetRenderWindow(mpPclVisualizer->getRenderWindow());
  mpPclVisualizer->setupInteractor(mpQVtkWidget->GetInteractor(), mpQVtkWidget->GetRenderWindow());
  mpQVtkWidget->update();

  mpPclVisualizer->setCameraPosition(0, 0, -1, 0, 0, 1, 0, -1, 0);
  mpPclVisualizer->setCameraFieldOfView(0.95);
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
  pcl::PolygonMesh cameraMesh;

  initzializeCameraMeshCloud(&cameraMesh.cloud);

  Eigen::Vector3f cameraShapeVertices[CAMERA_SHAPE_N_VERTICES];

  cameraShapeVertices[0] = Eigen::Vector3f( 0.f,  0.f, 0.f);
  cameraShapeVertices[1] = Eigen::Vector3f(-1.f, -1.f, 1.f);
  cameraShapeVertices[2] = Eigen::Vector3f(-1.f,  1.f, 1.f);
  cameraShapeVertices[3] = Eigen::Vector3f( 1.f, -1.f, 1.f);
  cameraShapeVertices[4] = Eigen::Vector3f( 1.f,  1.f, 1.f);

  Eigen::Vector3f cameraBasePos(position.x, position.y, position.z);
  Eigen::Quaternionf quat = rotation.normalized();
  Eigen::Matrix3f rotationMatrix = quat.toRotationMatrix();

  for(int i = 0; i < CAMERA_SHAPE_N_VERTICES; i++)
  {
    cameraShapeVertices[i] = rotationMatrix * cameraShapeVertices[i];
    cameraShapeVertices[i] += cameraBasePos;

    for(unsigned int j = 0; j < 3; j++)
    {
      reinterpret_cast<float*>(cameraMesh.cloud.data.data())[i * 3 + j] = cameraShapeVertices[i][j];
    }
  }

  pcl::Vertices v;

  v.vertices.push_back(1);
  v.vertices.push_back(2);
  v.vertices.push_back(4);
  v.vertices.push_back(3);
  cameraMesh.polygons.push_back(v);

  v.vertices.clear();
  v.vertices.push_back(0);
  v.vertices.push_back(1);
  v.vertices.push_back(3);
  v.vertices.push_back(0);
  cameraMesh.polygons.push_back(v);

  v.vertices.clear();
  v.vertices.push_back(0);
  v.vertices.push_back(2);
  v.vertices.push_back(4);
  v.vertices.push_back(0);
  cameraMesh.polygons.push_back(v);

  mpPclVisualizer->addPolylineFromPolygonMesh(cameraMesh, id.toStdString());
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
  mpPclVisualizer->removeShape(id.toStdString());
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

void CPclView::showCoordinateSystem(bool state)
{
  if(state && !mCoordinateSystemAdded)
  {
    mCoordinateSystemAdded = true;
    mpPclVisualizer->addCoordinateSystem();
  }
  else if(!state && mCoordinateSystemAdded)
  {
    mCoordinateSystemAdded = false;
    mpPclVisualizer->addCoordinateSystem();
  }
}

//Not implemented
/*
void CPclView::setBackground(float red, float blue, float green)
{

}

void CPclView::centerOnPoint(const pcl::PointXYZ& point)
{

}
*/

void CPclView::initzializeCameraMeshCloud(pcl::PCLPointCloud2* cloud)
{
  pcl::PCLPointField field;

  field.datatype = pcl::PCLPointField::FLOAT32;
  field.count = 1;

  field.name = "x";
  field.offset = 0;
  cloud->fields.push_back(field);

  field.name = "y";
  field.offset = 4;
  cloud->fields.push_back(field);

  field.name = "z";
  field.offset = 8;
  cloud->fields.push_back(field);

  cloud->width = CAMERA_SHAPE_N_VERTICES;
  cloud->height = 1;
  cloud->point_step = 3 * sizeof(float);
  cloud->row_step = CAMERA_SHAPE_N_VERTICES * 3 * sizeof(float);
  cloud->is_dense = true;
  cloud->data.resize(CAMERA_SHAPE_N_VERTICES * 3 * sizeof(float));
}
#endif
