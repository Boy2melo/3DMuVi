#include "acontextdatastore.h"
#include <QUuid>

AContextDataStore::AContextDataStore() {
    QUuid uuid = QUuid().createUuid();
    mContextId = uuid.toString();
    resetCalculationStep();
}

QString AContextDataStore::getId() const {
    return mContextId;
}

qint32 AContextDataStore::getCurrentCalculationStep() const {
    return mCalculationStep;
}

void AContextDataStore::Serialize() {
    // TODO: Prepare new context
    OnSerialize();
}

bool AContextDataStore::IsAborted() const {
    return mAborted;
}

void AContextDataStore::SetIsAborted(bool abort) {
    mAborted = abort;
}

void AContextDataStore::resetCalculationStep() {
    mCalculationStep = -1;
}

void AContextDataStore::incCalculationStep() {
    mCalculationStep++;
}
