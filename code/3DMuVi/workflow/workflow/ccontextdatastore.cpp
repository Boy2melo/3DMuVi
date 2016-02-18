#include "ccontextdatastore.h"
#include "io/CInputDataSet.h"
#include <QUuid>

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
    appendData(inputData, true);
}

QString CContextDataStore::getId() const {
    return mContextId;
}

qint32 CContextDataStore::getCurrentCalculationStep() const {
    return mCalculationStep;
}

void CContextDataStore::Serialize() {
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

    IDataPacket *reference;
    *reference = T();

    for(IDataPacket *packet : mDataPackets) {
        if(packet->getDataType() == reference->getDataType()) {
            return static_cast<T*>(packet);
        }
    }

    return nullptr;
}

template <typename T>
void CContextDataStore::appendData(T* data, bool overwrite) {
    // T muss von IDataPacket erben
    (void)static_cast<IDataPacket*>((T*)0);

    T* heap_obj = new T();
    *heap_obj = *data;

    auto reference = getData<T>();

    if(reference != nullptr && overwrite) {
        mDataPackets.removeAll(reference);
        delete reference;

        mDataPackets.push_back(heap_obj);
    } else if(reference == nullptr) {
        mDataPackets.push_back(heap_obj);
    }
}

