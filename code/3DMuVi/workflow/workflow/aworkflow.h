#ifndef AWORKFLOW_H
#define AWORKFLOW_H

#include "workflow/plugin/iplugin.h"
#include "workflow/workflow/acontextdatastore.h"
#include <QVector>

#define __PREPARE_ALGORITHM(STORE) AContextDataStore* store = (AContextDataStore *)STORE; store->SetIsAborted(false); store->resetCalculationStep();
#define __RUN_ALGORITHM(PLUGIN, CALLBACK) \
    IAlgorithm* algorithm = PLUGIN->getAlgorithm(); \
    if(!algorithm->IsBusy() && !store->IsAborted()){ \
        store->incCalculationStep(); \
        algorithm->run(store, [this](AContextDataStore *store){CALLBACK}); \
    }

class AWorkflow
{
public:
    virtual ~AWorkflow() {
    }

    virtual quint32 getStepCount() const = 0;
    virtual QString getAlgorithmType(const quint32 step) const = 0;
    virtual bool trySetStep(const quint32 step, IPlugin* plugin) = 0;
    virtual IPlugin* getStep(const quint32 step) const = 0;
    virtual QList<AContextDataStore*> getDataStores() const = 0;
    virtual AContextDataStore* addDataStore() = 0;
    virtual bool removeDataStore(QString id) = 0;

    virtual bool run(const QString storeId) = 0;
    virtual qint32 getState(const QString storeId) const = 0;
    virtual void stop(const QString storeId) = 0;
    virtual bool checkAvailableDataTypes() const = 0;
};

#endif // AWORKFLOW_H
