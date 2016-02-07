#include <vtkRenderWindow.h>

#include "CPclView.h"

CPclView::CPclView(QWidget *parent) : QWidget(parent)
{
  mpQVtkWidget.reset(new QVTKWidget(this));
  mpPclVisualizer.reset(new pcl::visualization::PCLVisualizer("", false));

  mpQVtkWidget->SetRenderWindow(mpPclVisualizer->getRenderWindow());
  mpPclVisualizer->setupInteractor(mpQVtkWidget->GetInteractor(), mpQVtkWidget->GetRenderWindow());
  mpQVtkWidget->update();
}

CPclView::~CPclView()
{

}

void CPclView::addPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud, uint32_t id)
{

}

void CPclView::addPolygonMesh(const pcl::PolygonMesh& mesh, uint32_t id)
{

}

void CPclView::addTextureMesh(const pcl::TextureMesh& mesh, uint32_t id)
{

}

void CPclView::addCameraMesh(pcl::PointXYZ position, Eigen::Vector4f rotation)
{

}

void CPclView::removePointCloud(uint32_t id)
{

}

void CPclView::removePolygonMesh(uint32_t id)
{

}

void CPclView::removeTextureMesh(uint32_t id)
{

}

void CPclView::removeCameraMesh(uint32_t id)
{

}

void CPclView::updatePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud, uint32_t id)
{

}

void CPclView::updatePolygonMesh(const pcl::PolygonMesh& mesh, uint32_t id)
{

}

void CPclView::updateTextureMesh(const pcl::TextureMesh& mesh, uint32_t id)
{

}

void CPclView::updateCameraMesh(pcl::PointXYZ position, Eigen::Vector4f rotation)
{

}

void CPclView::showCoordinateSystem(bool state)
{

}

void CPclView::setBackground(float red, float blue, float green)
{

}

void CPclView::centerOnPoint(pcl::PointXYZ point)
{

}
