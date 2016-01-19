#ifndef AWORKFLOW_H
#define AWORKFLOW_H

#include "workflow/plugin/aplugin.h"
#include "workflow/workflow/acontextdatastore.h"
#include <QVector>

class AWorkflow
{
public:
    AWorkflow();
    const quint32 getStepCount();
    QString getAlgorithmType(const quint32 step);
    bool trySetStep(const quint32 step, APlugin* plugin);
    APlugin* getStep(const quint32 step);
    QVector<AContextDataStore*> getDataStores();
    AContextDataStore* addDataStore();
    bool removeDataStore(QString id);

    void run(const QString storeId);
    quint32 getState(const QString storeId);
    void stop(const QString storeId);
    bool checkAvailableDataTypes();
};

#endif // AWORKFLOW_H
