#include "ccontextdatastore.h"
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

QString CContextDataStore::getId() const {
    return mContextId;
}

qint32 CContextDataStore::getCurrentCalculationStep() const {
    return mCalculationStep;
}

void CContextDataStore::Serialize() {
    // TODO: Prepare new context
    //OnSerialize();
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

void CContextDataStore::InitializeFromStorage() {}

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
        if(packet->getDataType() == ((IDataPacket)reference).getDataType()) {
            return packet;
        }
    }

    auto packet = new T();
    mDataPackets.push_back(static_cast<IDataPacket*>(packet));

    return packet;
}
