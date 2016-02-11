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

void CPclView::addPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud, const QString& id)
{

}

void CPclView::addPolygonMesh(const pcl::PolygonMesh& mesh, const QString& id)
{

}

void CPclView::addTextureMesh(const pcl::TextureMesh& mesh, const QString& id)
{

}

void CPclView::addCameraMesh(const pcl::PointXYZ& position, const Eigen::Vector4f& rotation,
                             const QString& id)
{

}

void CPclView::removePointCloud(const QString& id)
{

}

void CPclView::removePolygonMesh(const QString& id)
{

}

void CPclView::removeTextureMesh(const QString& id)
{

}

void CPclView::removeCameraMesh(const QString& id)
{

}

void CPclView::updatePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud,
                                const QString& id)
{

}

void CPclView::updatePolygonMesh(const pcl::PolygonMesh& mesh, const QString& id)
{

}

void CPclView::updateTextureMesh(const pcl::TextureMesh& mesh, const QString& id)
{

}

void CPclView::updateCameraMesh(const pcl::PointXYZ& position, const Eigen::Vector4f& rotation,
                                const QString& id)
{

}

void CPclView::showCoordinateSystem(bool state)
{

}

void CPclView::setBackground(float red, float blue, float green)
{

}

void CPclView::centerOnPoint(const pcl::PointXYZ& point)
{

}
