#ifndef IDATAVIEW_H
#define IDATAVIEW_H
#include "idatapacket.h"

class CInputDataSet;
class CDataFeature;
class CDataDepth;
class CDataPose;
#ifdef PCL
class CDataFusion;
#endif

class IDataView {
public:
    IDataView();

    void applyData(IDataPacket const *data);
    virtual void applyData(CInputDataSet const *) {}
    virtual void applyData(CDataFeature const *) {}
    virtual void applyData(CDataDepth const *) {}
    virtual void applyData(CDataPose const *) {}
#ifdef PCL
    virtual void applyData(CDataFusion const *) {}
#endif
};

#endif // IDATAVIEW_H
