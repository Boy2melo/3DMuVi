#pragma once
#include "workflow/workflow/acontextdatastore.h"
class CFourPhaseDataStore : AContextDataStore {
public:
    CFourPhaseDataStore();
    ~CFourPhaseDataStore();


protected:
    void OnSerialize() override;
public:
    QVector<QString> getSupportedDataTypes() const override;
    void InitializeFromStorage() override;
    void ApplyToDataView(IDataView *view) const override;
};

