#include "acontextdatastore.h"
#include <QUuid>

AContextDataStore::AContextDataStore()
{
    QUuid uuid = QUuid().createUuid();
    mContextId = uuid.toString();
}

QString AContextDataStore::getId() const{
    return mContextId;
}

qint32 AContextDataStore::getCurrentCalculationStep(){
    return mCalculationStep;
}

void AContextDataStore::Serialize() {
    // TODO: Prepare new context
    OnSerialize();
}
