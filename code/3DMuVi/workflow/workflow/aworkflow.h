#ifndef AWORKFLOW_H
#define AWORKFLOW_H

#include "workflow/plugin/iplugin.h"
#include "workflow/workflow/acontextdatastore.h"
#include <QVector>

class AWorkflow
{
public:
    AWorkflow();
    quint32 getStepCount() const;
    QString getAlgorithmType(const quint32 step) const;
    bool trySetStep(const quint32 step, IPlugin* plugin);
    IPlugin* getStep(const quint32 step) const;
    QVector<AContextDataStore*> getDataStores() const;
    AContextDataStore* addDataStore();
    bool removeDataStore(QString id);

    void run(const QString storeId);
    quint32 getState(const QString storeId) const;
    void stop(const QString storeId);
    bool checkAvailableDataTypes() const;
};

#endif // AWORKFLOW_H
