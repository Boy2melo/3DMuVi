#include "ccontextdatastore.h"
#include "io/CInputDataSet.h"
#include <QUuid>
#include <io/CResultContext.h>
#include "datapackets/CDataFeature.h"
#include "datapackets/CDataDepth.h"
#include "datapackets/CDataPose.h"
#include "datapackets/CDataFusion.h"

CContextDataStore::~CContextDataStore() {}

CContextDataStore::CContextDataStore() {
    QUuid uuid = QUuid().createUuid();
    mContextId = uuid.toString();
    resetCalculationStep();
}

void CContextDataStore::InitializeFromStorage(CInputDataSet* inputData) {
    appendData(std::shared_ptr<CInputDataSet>(inputData));
}

QString CContextDataStore::getId() const {
    return mContextId;
}

qint32 CContextDataStore::getCurrentCalculationStep() const {
    return mCalculationStep;
}

void CContextDataStore::Serialize(CResultContext *context) {
    for(auto packet : mDataPackets) {
        context->addDataPacket(packet.get());
    }
}

bool CContextDataStore::IsAborted() const {
    return mAborted;
}

void CContextDataStore::SetIsAborted(bool abort) {
    mAborted = abort;
}

void CContextDataStore::resetCalculationStep() {
    mCalculationStep = -1;
}

void CContextDataStore::incCalculationStep() {
    mCalculationStep++;
}

void CContextDataStore::ApplyToDataView(IDataView* view) const {
    for (auto packet : mDataPackets) {
        packet->ApplyToDataview(view);
    }
}

template <typename T>
std::shared_ptr<T> CContextDataStore::getData() {
    // T muss von IDataPacket erben
    (void)static_cast<IDataPacket*>((T*)0);

    T reference;

    for(auto packet : mDataPackets) {
        if(packet->getDataType() == reference.getDataType()) {
            return std::dynamic_pointer_cast<T>(packet);
        }
    }

    return nullptr;
}

template <typename T>
std::shared_ptr<T> CContextDataStore::createData(bool overwrite) {
    // T muss von IDataPacket erben
    (void)static_cast<IDataPacket*>((T*)0);

    std::shared_ptr<T> obj();
    
    if(appendData(obj, overwrite)) {
        return obj;
    } else {
        return nullptr;
    }
}

template <typename T>
bool CContextDataStore::appendData(std::shared_ptr<T> data, bool overwrite) {
    // T muss von IDataPacket erben
    (void)static_cast<IDataPacket*>((T*)0);
    
    auto reference = getData<T>();

    if (reference != nullptr && overwrite) {
        mDataPackets.removeAll(reference);
        mDataPackets.push_back(data);
        return true;
    } else if (reference == nullptr) {
        mDataPackets.push_back(data);
        return false;
    }

    return false;
}

//Compiler muss Template Implementierungen anlegen, damit diese von den Plugins aufrufbar sind
template bool CContextDataStore::appendData<CDataFeature>(std::shared_ptr<CDataFeature>,bool);
template bool CContextDataStore::appendData<CDataDepth>(std::shared_ptr<CDataDepth>, bool);
template bool CContextDataStore::appendData<CDataPose>(std::shared_ptr<CDataPose>, bool);
//TODO: merge CDataFusion from pcl branch
#if PCL
template bool CContextDataStore::appendData<CDataFusion>(std::shared_ptr<CDataFusion>, bool);
#endif