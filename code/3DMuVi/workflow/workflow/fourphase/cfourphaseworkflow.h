#ifndef _H_CFOURPHASEWORKFLOW
#define _H_CFOURPHASEWORKFLOW

#include <workflow/workflow/aworkflow.h>
#include "workflow/plugin/iplugin.h"
#include "workflow/workflow/ccontextdatastore.h"

class CFourPhaseWorkflow : public AWorkflow {
private:
    IPlugin **mPlugins;

    private slots:
    // ReSharper disable once CppFunctionIsNotImplemented
    void SlotAlgorithmFinished(CContextDataStore *);

public:
    CFourPhaseWorkflow();

    quint32 getStepCount() const override;
    QString getAlgorithmType(const quint32 step) const override;
    bool trySetStep(const quint32 step, IPlugin* plugin) override;
    IPlugin* getStep(const quint32 step) const override;
    bool checkAvailableDataTypes() const override;

protected:
    void executeAlgorithm(CContextDataStore* store) override;
};

#endif