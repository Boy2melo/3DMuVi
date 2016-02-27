#include "io/CInputDataSet.h"
#include "datapackets/CDataFeature.h"
#include "datapackets/CDataDepth.h"
#include "datapackets/CDataPose.h"
#include "datapackets/CDataFusion.h"

#include "idataview.h"

IDataView::IDataView()
{
}

void IDataView::applyData(const IDataPacket *data)
{
    if(data->getDataType() == DT_INPUTIMAGES)
    {
      applyData(static_cast<CInputDataSet const*>(data));
    }
    else if(data->getDataType() == DT_FEATURE_MATCH)
    {
      applyData(static_cast<CDataFeature const*>(data));
    }
    else if(data->getDataType() == DT_DEPTH)
    {
      applyData(static_cast<CDataDepth const*>(data));
    }
    else if(data->getDataType() == DT_POSE)
    {
      applyData(static_cast<CDataPose const*>(data));
    }
    //TODO: merge CDataFusion from pcl branch
#if PCL
    else if(data->getDataType() == DT_FUSION)
    {
      applyData(static_cast<CDataFusion const*>(data));
    }
#endif
}
