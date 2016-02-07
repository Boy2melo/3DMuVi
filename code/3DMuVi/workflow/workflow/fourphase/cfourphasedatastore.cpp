#include "cfourphasedatastore.h"

CFourPhaseDataStore::CFourPhaseDataStore() {}

CFourPhaseDataStore::~CFourPhaseDataStore() {}


void CFourPhaseDataStore::OnSerialize() {}

QStringList CFourPhaseDataStore::getSupportedDataTypes() const {
    return QStringList();
}

void CFourPhaseDataStore::InitializeFromStorage() {}

void CFourPhaseDataStore::ApplyToDataView(IDataView *view) const {}
