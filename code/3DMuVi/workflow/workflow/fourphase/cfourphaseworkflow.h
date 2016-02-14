#pragma once
#include <workflow/workflow/aworkflow.h>
#include "workflow/plugin/iplugin.h"
#include "cfourphasedatastore.h"

class CFourPhaseWorkflow : public AWorkflow {
private:
    IPlugin **mPlugins;
    QList<CFourPhaseDataStore *>* mDataStores;

        private slots:
    void SlotAlgorithmFinished(AContextDataStore *);

public:
    CFourPhaseWorkflow();

    quint32 getStepCount() const override;
    QString getAlgorithmType(const quint32 step) const override;
    bool trySetStep(const quint32 step, IPlugin* plugin) override;
    IPlugin* getStep(const quint32 step) const override;
    QList<AContextDataStore*> getDataStores() const override;
    AContextDataStore* addDataStore() override;
    bool removeDataStore(QString id) override;
    qint32 getState(const QString storeId) const override;
    void stop(const QString storeId) override;
    bool checkAvailableDataTypes() const override;

protected:
    AContextDataStore *FindStore(QString id) const override;
    void executeAlgorithm(AContextDataStore* store) override;
};

