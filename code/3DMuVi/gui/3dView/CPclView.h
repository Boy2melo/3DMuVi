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

  void addPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud, uint32_t id);
  void addPolygonMesh(const pcl::PolygonMesh& mesh, uint32_t id);
  void addTextureMesh(const pcl::TextureMesh& mesh, uint32_t id);
  void addCameraMesh(pcl::PointXYZ position, Eigen::Vector4f rotation);

  void removePointCloud(uint32_t id);
  void removePolygonMesh(uint32_t id);
  void removeTextureMesh(uint32_t id);
  void removeCameraMesh(uint32_t id);

  void updatePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud, uint32_t id);
  void updatePolygonMesh(const pcl::PolygonMesh& mesh, uint32_t id);
  void updateTextureMesh(const pcl::TextureMesh& mesh, uint32_t id);
  void updateCameraMesh(pcl::PointXYZ position, Eigen::Vector4f rotation);

  void showCoordinateSystem(bool state);
  void setBackground(float red, float blue, float green);

  void centerOnPoint(pcl::PointXYZ point);

signals:
  void pointSelected(pcl::PointXYZ point);
  void cameraSelected(pcl::PointXYZ position, Eigen::Vector4f rotation);

private:
  std::unique_ptr<QVTKWidget> mpQVtkWidget;
  std::unique_ptr<pcl::visualization::PCLVisualizer> mpPclVisualizer;
};

#endif // CPCLVIEW_H
