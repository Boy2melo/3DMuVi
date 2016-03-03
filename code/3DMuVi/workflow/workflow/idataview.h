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

    /**
    @brief Castet das packet und ruft die spezialisierte Methode auf
    */
    void applyData(IDataPacket const *data);
    /**
    @brief Wendet daten vom Typ [CInputDataSet](@ref CInputDataSet) an
    */
    virtual void applyData(CInputDataSet const *) {}
    /**
    @brief Wendet daten vom Typ [CDataFeature](@ref CDataFeature) an
    */
    virtual void applyData(CDataFeature const *) {}
    /**
    @brief Wendet daten vom Typ [CDataDepth](@ref CDataDepth) an
    */
    virtual void applyData(CDataDepth const *) {}
    /**
    @brief Wendet daten vom Typ [CDataPose](@ref CDataPose) an
    */
    virtual void applyData(CDataPose const *) {}
#ifdef PCL
    /**
    @brief Wendet daten vom Typ [CDataFusion](@ref CDataFusion) an
    */
    virtual void applyData(CDataFusion const *) {}
#endif
};

#endif // IDATAVIEW_H
