#ifndef IDATAVIEW_H
#define IDATAVIEW_H
#include "idatapacket.h"

class CInputDataSet;
class CDataFeature;
class CDataDepth;
class CDataPose;
class CDataFusion;

class IDataView {
public:
    IDataView();

    void applyData(IDataPacket const *data);
    virtual void applyData(CInputDataSet const *) {}
    virtual void applyData(CDataFeature const *) {}
    virtual void applyData(CDataDepth const *) {}
    virtual void applyData(CDataPose const *) {}
    virtual void applyData(CDataFusion const *) {}
};

#endif // IDATAVIEW_H
