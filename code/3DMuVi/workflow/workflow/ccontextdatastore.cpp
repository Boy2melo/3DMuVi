#include "ccontextdatastore.h"
#include "io/CInputDataSet.h"
#include <QUuid>
#include <io/CResultContext.h>
#include "datapackets/CDataFeature.h"
#include "datapackets/CDataDepth.h"
#include "datapackets/CDataPose.h"
#include "datapackets/CDataFusion.h"

CContextDataStore::~CContextDataStore() {
    for(IDataPacket *packet : mDataPackets) {
        delete packet;
    }
}

CContextDataStore::CContextDataStore() {
    QUuid uuid = QUuid().createUuid();
    mContextId = uuid.toString();
    resetCalculationStep();
}

void CContextDataStore::InitializeFromStorage(CInputDataSet* inputData) {
    appendData(inputData);
}

QString CContextDataStore::getId() const {
    return mContextId;
}

qint32 CContextDataStore::getCurrentCalculationStep() const {
    return mCalculationStep;
}

void CContextDataStore::Serialize(CResultContext *context) {
    for(IDataPacket *packet : mDataPackets) {
        context->addDataPacket(packet);
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
    for (IDataPacket *packet : mDataPackets) {
        packet->ApplyToDataview(view);
    }
}

template <typename T>
T* CContextDataStore::getData() {
    // T muss von IDataPacket erben
    (void)static_cast<IDataPacket*>((T*)0);

    T reference;

    for(IDataPacket *packet : mDataPackets) {
        if(packet->getDataType() == reference.getDataType()) {
            return static_cast<T*>(packet);
        }
    }

    return nullptr;
}

template <typename T>
T* CContextDataStore::createData(bool overwrite) {
    // T muss von IDataPacket erben
    (void)static_cast<IDataPacket*>((T*)0);

    T* heap_obj = new T();

    if(appendData(heap_obj, overwrite)) {
        return heap_obj;
    } else {
        delete heap_obj;
        return nullptr;
    }
}

template <typename T>
bool CContextDataStore::appendData(T* data, bool overwrite) {
    // T muss von IDataPacket erben
    (void)static_cast<IDataPacket*>((T*)0);
    
    auto reference = getData<T>();

    if (reference != nullptr && overwrite) {
        mDataPackets.removeAll(reference);
        delete reference;

        mDataPackets.push_back(data);
        return true;
    } else if (reference == nullptr) {
        mDataPackets.push_back(data);
        return false;
    }

    return false;
}

//Compiler muss Template Implementierungen anlegen, damit diese von den Plugins aufrufbar sind
template bool CContextDataStore::appendData<CDataFeature>(CDataFeature*,bool);
template bool CContextDataStore::appendData<CDataDepth>(CDataDepth*, bool);
template bool CContextDataStore::appendData<CDataPose>(CDataPose*, bool);
//TODO: merge CDataFusion from pcl branch
//template bool CContextDataStore::appendData<CDataFusion>(CDataFusion*, bool);
