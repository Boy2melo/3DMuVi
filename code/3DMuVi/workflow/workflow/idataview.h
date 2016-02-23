#ifndef IDATAVIEW_H
#define IDATAVIEW_H
#include "idatapacket.h"
#include "io/CInputDataSet.h"

#ifdef PCL
#include "datapackets/CDataFusion.h"
#endif

class CDataPose;
class CDataDepth;

class IDataView {
public:
    IDataView();

    void applyData(IDataPacket const *data) {}
    void applyData(CInputDataSet const *data) {}
    void applyData(CDataFeature const *data) {}
    void applyData(CDataDepth const *data) {}
    void applyData(CDataPose const *data) {}
#ifdef PCL
    void applyData(CDataFusion const *data) {}
#endif
};

#endif // IDATAVIEW_H
