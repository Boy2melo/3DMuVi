#ifndef CPCLVIEW_H
#define CPCLVIEW_H

#include <QWidget>

#include <QVTKWidget.h>

#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>
#include <pcl/visualization/pcl_visualizer.h>

/*!
\brief This class gives a simple interface for viewing 3D objects.
\author Stefan Wolf
*/
class CPclView : public QWidget
{
  Q_OBJECT

public:
  /*!
  \brief Default constructor.
  \param parent The widget which should be the parent of this one.
  */
  explicit CPclView(QWidget *parent = nullptr);

  /*!
   * \brief Default destructor.
   */
  virtual ~CPclView();

  /*!
   * \brief Adds a point cloud to the view.
   * \param pointCloud The point cloud to add.
   * \param id The id which is later used to update or remove the point cloud.
   */
  void addPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud,
                     const QString& id = QString("cloud"));

  /*!
   * \brief Adds a polygon mesh to the view.
   * \param mesh The polygon mesh to add.
   * \param id The id which is later used to update or remove the polygon mesh.
   */
  void addPolygonMesh(const pcl::PolygonMesh& mesh, const QString& id = QString("polymesh"));

  /*!
   * \brief Adds a texture mesh to the view.
   * \param mesh The texture mesh to add.
   * \param id The id which is later used to update or remove the texture mesh.
   */
  void addTextureMesh(const pcl::TextureMesh& mesh, const QString& id = QString("texmesh"));

  /*!
   * \brief Adds a camera mesh to the view.
   * \param position The camera's position.
   * \param rotation The camera's view angle.
   * \param id The id which is later used to update or remove the camera.
   */
  void addCameraMesh(const pcl::PointXYZ& position, const Eigen::Quaternionf& rotation,
                     const QString& id = QString("camera"));

  /*!
   * \brief Removes the point cloud with the given id from the view.
   * \param id The point cloud's id.
   */
  void removePointCloud(const QString& id = QString("cloud"));

  /*!
   * \brief Removes the polygon mesh with the given id from the view.
   * \param id The polygon mesh's id.
   */
  void removePolygonMesh(const QString& id = QString("polymesh"));

  /*!
   * \brief Removes the texture mesh with the given id from the view.
   * \param id The texture mesh's id.
   */
  void removeTextureMesh(const QString& id = QString("texmesh"));

  /*!
   * \brief Removes the camera mesh with the given id from the view.
   * \param id The camera's id.
   */
  void removeCameraMesh(const QString& id = QString("camera"));

  /*!
   * \brief Updates the point cloud with the given id.
   * \param pointCloud The new point cloud.
   * \param id The id of the point cloud which should be updated.
   */
  void updatePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud,
                        const QString& id = QString("cloud"));

  /*!
   * \brief Updates the polygon mesh with the given id.
   * \param mesh The new polygon mesh.
   * \param id The id of the polygon mesh which should be updated.
   */
  void updatePolygonMesh(const pcl::PolygonMesh& mesh, const QString& id = QString("polymesh"));

  /*!
   * \brief Updates the texture mesh with the given id.
   * \param mesh The new texture mesh.
   * \param id The id of the texture mesh which should be updated.
   */
  void updateTextureMesh(const pcl::TextureMesh& mesh, const QString& id = QString("texmesh"));

  /*!
   * \brief Updates the camera mesh with the given id.
   * \param position The new camera's position.
   * \param rotation The new camera's view angle.
   * \param id The id of the camera which should be updated.
   */
  void updateCameraMesh(const pcl::PointXYZ& position, const Eigen::Quaternionf& rotation,
                        const QString& id = QString("camera"));

  /*!
  \brief Not implemented.
  */
  void showCoordinateSystem(bool state);

  /*!
  \brief Not implemented.
  */
  void setBackground(float red, float blue, float green);

  /*!
  \brief Not implemented.
  */
  void centerOnPoint(const pcl::PointXYZ& point);

signals:
  /*!
  \brief Not implemented.
  */
  void pointSelected(const pcl::PointXYZ& point);

  /*!
  \brief Not implemented.
  */
  void cameraSelected(const pcl::PointXYZ& position, const Eigen::Vector4f& rotation);

private:
  std::unique_ptr<QVTKWidget> mpQVtkWidget;
  std::unique_ptr<pcl::visualization::PCLVisualizer> mpPclVisualizer;
};

#endif // CPCLVIEW_H
