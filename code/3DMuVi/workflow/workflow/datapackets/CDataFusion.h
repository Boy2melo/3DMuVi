#ifdef PCL
#ifndef CDATAFUSION_H
#define CDATAFUSION_H

#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>

#include "macros.h"
#include "workflow/workflow/idatapacket.h"

using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;

/*!
 * \class CDataFusion
 * \brief The CDataFusion class
 * \author Stefan Wolf
 */
class EXPORTED CDataFusion : public IDataPacket, public std::enable_shared_from_this<CDataFusion>
{
public:
  /*!
   * \brief Returns DT_FUSION as the data type of this object.
   * \return DT_FUSION.
   */
  QString getDataType() const override;

  /*!
   * \brief Returns a new stream provider.
   * \return A new stream provider.
   */
  std::unique_ptr<AStreamProvider> getStreamProvider() override;

  /*!
   * \brief Serializes the data of this object into the given stream.
   * \param stream The stream in which the data is written.
   */
  void serialize(AStreamProvider* stream) override;

  /*!
   * \brief Deserializes the data from the given stream into this object.
   * \param stream The stream from which the data is read.
   */
  void deserialize(AStreamProvider *stream) override;

  /*!
   * \brief Sets the point cloud.
   * \param cloud The point cloud which should be kept by this data packet.
   */
  void setPointCloud(PointCloud::Ptr cloud);

  /*!
   * \brief Returns the point cloud.
   * \return The point cloud which is kept by this data packet.
   */
  PointCloud::Ptr getPointCloud() const;

  /*!
   * \brief Sets the polygon mesh.
   * \param mesh The polygon mesh which should be kept by this data packet.
   */
  void setPolygonMesh(pcl::PolygonMesh::Ptr mesh);

  /*!
   * \brief Returns the polygon mesh.
   * \return The polygon mesh which is kept by this data packet.
   */
  pcl::PolygonMesh::Ptr getPolygonMesh() const;

  /*!
   * \brief Sets the texture mesh.
   * \param mesh The texture mesh which should be kept by this data packet.
   */
  void setTextureMesh(pcl::TextureMesh::Ptr mesh);

  /*!
   * \brief Returns the texture mesh.
   * \return The texture mesh which is kept by this data packet.
   */
  pcl::TextureMesh::Ptr getTextureMesh() const;

  void applyToDataview(IDataView* dataView) const override;

private:
  PointCloud::Ptr mPointCloudData;
  pcl::PolygonMesh::Ptr mPolygonMeshData;
  pcl::TextureMesh::Ptr mTextureMeshData;
  AStreamProvider* mpStream = nullptr;
};

#endif // CDATAFUSION_H
#endif // PCL
