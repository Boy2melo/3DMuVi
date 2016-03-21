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
  @brief Castet das packet und ruft die spezialisierte Methode auf.
  @param data Das Packet, das auf diese View angewendet werden soll.
  */
  void applyData(IDataPacket const* data);
  /**
  @brief Wendet Daten vom Typ [CInputDataSet](@ref CInputDataSet) an.
  */
  virtual void applyData(CInputDataSet const*) {}
  /**
  @brief Wendet Daten vom Typ [CDataFeature](@ref CDataFeature) an.
  */
  virtual void applyData(CDataFeature const*) {}
  /**
  @brief Wendet Daten vom Typ [CDataDepth](@ref CDataDepth) an.
  */
  virtual void applyData(CDataDepth const*) {}
  /**
  @brief Wendet Daten vom Typ [CDataPose](@ref CDataPose) an.
  */
  virtual void applyData(CDataPose const*) {}
#ifdef PCL
  /**
  @brief Wendet Daten vom Typ [CDataFusion](@ref CDataFusion) an.
  */
  virtual void applyData(CDataFusion const*) {}
#endif
};

#endif // IDATAVIEW_H
