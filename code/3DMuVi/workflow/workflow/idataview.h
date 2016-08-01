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

class IDataView
{
public:
  /**
  @brief Leerer Konstruktor.
  */
  IDataView();

  /**
  @brief Löscht alle bisher angewendeten Daten.
  */
  virtual void clearData() {}
  /**
  @brief Wendet Daten vom Typ [CInputDataSet](@ref CInputDataSet) an.
  */
  virtual void applyData(std::shared_ptr<CInputDataSet const>) {}
  /**
  @brief Wendet Daten vom Typ [CDataFeature](@ref CDataFeature) an.
  */
  virtual void applyData(std::shared_ptr<CDataFeature const>) {}
  /**
  @brief Wendet Daten vom Typ [CDataDepth](@ref CDataDepth) an.
  */
  virtual void applyData(std::shared_ptr<CDataDepth const>) {}
  /**
  @brief Wendet Daten vom Typ [CDataPose](@ref CDataPose) an.
  */
  virtual void applyData(std::shared_ptr<CDataPose const>) {}
#ifdef PCL
  /**
  @brief Wendet Daten vom Typ [CDataFusion](@ref CDataFusion) an.
  */
  virtual void applyData(std::shared_ptr<CDataFusion const>) {}
#endif
};

#endif // IDATAVIEW_H
