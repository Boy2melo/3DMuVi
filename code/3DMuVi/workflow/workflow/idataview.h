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
    virtual void applyData(CInputDataSet const * /* data */) {}
    virtual void applyData(CDataFeature const * /* data */) {}
    virtual void applyData(CDataDepth const * /* data */) {}
    virtual void applyData(CDataPose const * /* data */) {}
    virtual void applyData(CDataFusion const * /* data */) {}
};

#endif // IDATAVIEW_H
