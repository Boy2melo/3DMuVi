#ifndef IDATAVIEW_H
#define IDATAVIEW_H
#include "idatapacket.h"
#include "io/CInputDataSet.h"
#include "datapackets/CDataFusion.h"

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
    void applyData(CDataFusion const *data) {}
};

#endif // IDATAVIEW_H
