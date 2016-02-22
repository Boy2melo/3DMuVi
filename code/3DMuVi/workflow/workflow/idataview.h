#ifndef IDATAVIEW_H
#define IDATAVIEW_H
#include "idatapacket.h"
#include "io/CInputDataSet.h"
#include "datapackets/CDataDepth.h"
#include "datapackets/CDataFusion.h"
#include "datapackets/CDataPose.h"

class IDataView {
public:
    IDataView();

    void applyData(IDataPacket *data) {}
    void applyData(CInputDataSet *data) {}
    void applyData(CDataFeature *data) {}
    void applyData(CDataDepth *data) {}
    void applyData(CDataPose *data) {}
    void applyData(CDataFusion *data) {}
};

#endif // IDATAVIEW_H
