#include "cfourphasedatastore.h"

CFourPhaseDataStore::CFourPhaseDataStore() {}

CFourPhaseDataStore::~CFourPhaseDataStore() {}


void CFourPhaseDataStore::OnSerialize() {}

QStringList CFourPhaseDataStore::getSupportedDataTypes() const {
    QStringList list;
    return list;
}

void CFourPhaseDataStore::InitializeFromStorage() {}

void CFourPhaseDataStore::ApplyToDataView(IDataView *view) const {}
