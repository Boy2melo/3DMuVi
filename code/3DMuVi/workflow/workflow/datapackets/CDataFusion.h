#ifndef CDATAFUSION_H
#define CDATAFUSION_H

#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>

#include "workflow/workflow/idatapacket.h"

using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;

/*!
 * \class CDataFusion
 * \brief The CDataFusion class
 * \author Stefan Wolf
 */
class CDataFusion : public IDataPacket
{
public:
  CDataFusion();
  virtual ~CDataFusion();

  QString getDataType() const override;
  AStreamProvider* getStreamProvider() override;
  void serialize(AStreamProvider* stream) override;
  void deserialize(AStreamProvider *stream) override;
  void setPointClound(PointCloud::Ptr cloud);
  PointCloud::Ptr getPointCloud();

private:
  PointCloud::Ptr mPointCloudData;
  AStreamProvider* mpStream;
};

#endif // CDATAFUSION_H
