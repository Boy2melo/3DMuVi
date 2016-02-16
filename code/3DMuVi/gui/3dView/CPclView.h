#ifndef CPCLVIEW_H
#define CPCLVIEW_H

#include <QWidget>

#include <QVTKWidget.h>

#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>
#include <pcl/visualization/pcl_visualizer.h>

class CPclView : public QWidget
{
  Q_OBJECT

public:
  explicit CPclView(QWidget *parent = nullptr);
  virtual ~CPclView();

  void addPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud,
                     const QString& id = QString("cloud"));
  void addPolygonMesh(const pcl::PolygonMesh& mesh, const QString& id = QString("polymesh"));
  void addTextureMesh(const pcl::TextureMesh& mesh, const QString& id = QString("texmesh"));
  void addCameraMesh(const pcl::PointXYZ& position, const Eigen::Quaternionf& rotation,
                     const QString& id = QString("camera"));

  void removePointCloud(const QString& id = QString("cloud"));
  void removePolygonMesh(const QString& id = QString("polymesh"));
  void removeTextureMesh(const QString& id = QString("texmesh"));
  void removeCameraMesh(const QString& id = QString("camera"));

  void updatePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud,
                        const QString& id = QString("cloud"));
  void updatePolygonMesh(const pcl::PolygonMesh& mesh, const QString& id = QString("polymesh"));
  void updateTextureMesh(const pcl::TextureMesh& mesh, const QString& id = QString("texmesh"));
  void updateCameraMesh(const pcl::PointXYZ& position, const Eigen::Quaternionf& rotation,
                        const QString& id = QString("camera"));

  void showCoordinateSystem(bool state);
  void setBackground(float red, float blue, float green);

  void centerOnPoint(const pcl::PointXYZ& point);

signals:
  void pointSelected(const pcl::PointXYZ& point);
  void cameraSelected(const pcl::PointXYZ& position, const Eigen::Vector4f& rotation);

private:
  std::unique_ptr<QVTKWidget> mpQVtkWidget;
  std::unique_ptr<pcl::visualization::PCLVisualizer> mpPclVisualizer;
};

#endif // CPCLVIEW_H
