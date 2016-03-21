#include "ccontextdatastore.h"
#include "io/CInputDataSet.h"
#include <QUuid>
#include <io/CResultContext.h>
#include "datapackets/CDataFeature.h"
#include "datapackets/CDataDepth.h"
#include "datapackets/CDataPose.h"
#ifdef PCL
#include "datapackets/CDataFusion.h"
#endif

CContextDataStore::~CContextDataStore() { }

CContextDataStore::CContextDataStore()
{
  QUuid uuid = QUuid().createUuid();
  mContextId = uuid.toString();
  resetCalculationStep();
  mAborted = false;
}

void CContextDataStore::InitializeFromStorage(CInputDataSet* inputData)
{
  appendData(std::shared_ptr<CInputDataSet>(inputData));
}

QString CContextDataStore::getId() const
{
  return mContextId;
}

qint32 CContextDataStore::getCurrentCalculationStep() const
{
  return mCalculationStep;
}

void CContextDataStore::Serialize(CResultContext* context)
{
  for(auto packet : mDataPackets)
  {
    context->addDataPacket(packet);
  }
}

bool CContextDataStore::IsAborted() const
{
  return mAborted;
}

void CContextDataStore::SetIsAborted(bool abort)
{
  mAborted = abort;
}

void CContextDataStore::resetCalculationStep()
{
  mCalculationStep = -1;
}

void CContextDataStore::incCalculationStep()
{
  mCalculationStep++;
}

void CContextDataStore::ApplyToDataView(IDataView* view) const
{
  view->clearData();
  for(auto packet : mDataPackets)
  {
    packet->ApplyToDataview(view);
  }
}

template <typename T>
std::shared_ptr<T> CContextDataStore::getData()
{
  // T muss von IDataPacket erben
  (void)static_cast<IDataPacket*>((T*)0);

  T reference;

  for(auto packet : mDataPackets)
  {
    if(packet->getDataType() == reference.getDataType())
    {
      return std::dynamic_pointer_cast<T>(packet);
    }
  }

  return nullptr;
}

template <typename T>
std::shared_ptr<T> CContextDataStore::createData(bool overwrite)
{
  // T muss von IDataPacket erben
  (void)static_cast<IDataPacket*>((T*)0);

  std::shared_ptr<T> obj = std::make_shared<T>();

  if(appendData(obj, overwrite))
  {
    return obj;
  }
  else
  {
    return nullptr;
  }
}

template <typename T>
bool CContextDataStore::appendData(std::shared_ptr<T> data, bool overwrite)
{
  // T muss von IDataPacket erben
  (void)static_cast<IDataPacket*>((T*)0);

  auto reference = getData<T>();

  if(reference != nullptr && overwrite)
  {
    mDataPackets.removeAll(reference);
    mDataPackets.push_back(data);
    return true;
  }
  else if(reference == nullptr)
  {
    mDataPackets.push_back(data);
    return true;
  }

  return false;
}

//Compiler muss Template Implementierungen anlegen, damit diese von den Plugins aufrufbar sind
template EXPORTED bool CContextDataStore::appendData<CInputDataSet>(std::shared_ptr<CInputDataSet>,
                                                                    bool);
template EXPORTED bool CContextDataStore::appendData<CDataFeature>(std::shared_ptr<CDataFeature>,
                                                                   bool);
template EXPORTED bool CContextDataStore::appendData<CDataDepth>(std::shared_ptr<CDataDepth>, bool);
template EXPORTED bool CContextDataStore::appendData<CDataPose>(std::shared_ptr<CDataPose>, bool);
template EXPORTED std::shared_ptr<CInputDataSet> CContextDataStore::createData<CInputDataSet>(bool);
template EXPORTED std::shared_ptr<CDataFeature> CContextDataStore::createData<CDataFeature>(bool);
template EXPORTED std::shared_ptr<CDataDepth> CContextDataStore::createData<CDataDepth>(bool);
template EXPORTED std::shared_ptr<CDataPose> CContextDataStore::createData<CDataPose>(bool);
template EXPORTED std::shared_ptr<CInputDataSet> CContextDataStore::getData<CInputDataSet>();
template EXPORTED std::shared_ptr<CDataFeature> CContextDataStore::getData<CDataFeature>();
template EXPORTED std::shared_ptr<CDataDepth> CContextDataStore::getData<CDataDepth>();
template EXPORTED std::shared_ptr<CDataPose> CContextDataStore::getData<CDataPose>();
#if PCL
template EXPORTED bool CContextDataStore::appendData<CDataFusion>(std::shared_ptr<CDataFusion>,
                                                                  bool);
template EXPORTED std::shared_ptr<CDataFusion> CContextDataStore::createData<CDataFusion>(bool);
template EXPORTED std::shared_ptr<CDataFusion> CContextDataStore::getData<CDataFusion>();
#endif
