#include "cfourphaseworkflow.h"
#include "workflow/plugin/cpluginmanager.h"
#include "logger/controll/CLogController.h"

CFourPhaseWorkflow::CFourPhaseWorkflow() : AWorkflow() {
    mPlugins = new IPlugin*[getStepCount()];

    for (quint32 i = 0; i < getStepCount(); i++) {
        mPlugins[i] = nullptr;
    }
}

CFourPhaseWorkflow::~CFourPhaseWorkflow()
{
    delete [] mPlugins;
}


quint32 CFourPhaseWorkflow::getStepCount() const {
    return 4;
}

QString CFourPhaseWorkflow::getAlgorithmType(const quint32 step) const {
    switch (step) {
    case 0:
        return PT_FeatureMatcher;
    case 1:
        return PT_PoseEstimator;
    case 2:
        return PT_DepthEstimator;
    case 3:
        return PT_Fusion;
    default:
        return "";
    }
}

bool CFourPhaseWorkflow::trySetStep(const quint32 step, IPlugin* plugin) {
    if (step >= getStepCount()) {
        return false;
    } else {
        if (plugin == nullptr || plugin->GetPluginType() == getAlgorithmType(step)) {
            mPlugins[step] = plugin;
            return true;
        } else {
            return false;
        }
    }
}

IPlugin* CFourPhaseWorkflow::getStep(const quint32 step) const {
    if (step >= getStepCount()) {
        return nullptr;
    } else {
        return mPlugins[step];
    }
}

void CFourPhaseWorkflow::executeAlgorithm(CContextDataStore* store) {
    if (mPlugins[0]->getAlgorithm()->IsBusy()) {
        return;
    }

#define DEBUG_MACRO
#ifndef DEBUG_MACRO
    __RUN_ALGORITHM(mPlugins[0],
        __RUN_ALGORITHM(mPlugins[1],
            __RUN_ALGORITHM(mPlugins[2],
                __RUN_ALGORITHM(mPlugins[3], emit sigDataStoreFinished(store); ))));
#else
    IAlgorithm* algorithm = mPlugins[0]->getAlgorithm();
    algorithm->setLogger(&CLogController::instance());

    if (!algorithm->IsBusy() && !store->IsAborted()) {
        store->incCalculationStep();
        QString finishedM = mPlugins[0]->Name();
        finishedM.append(" - plugin started");
        CLogController::instance().frameworkMessage(finishedM);
        algorithm->run(store, [this](CContextDataStore *store) {
            IAlgorithm* algorithm = mPlugins[1]->getAlgorithm();
            algorithm->setLogger(&CLogController::instance());

            if (!algorithm->IsBusy() && !store->IsAborted()) {
                store->incCalculationStep();
                QString finishedM = mPlugins[1]->Name();
                finishedM.append(" - plugin started");
                CLogController::instance().frameworkMessage(finishedM);
                algorithm->run(store, [this](CContextDataStore *store) {
                    IAlgorithm* algorithm = mPlugins[2]->getAlgorithm();
                    algorithm->setLogger(&CLogController::instance());

                    if (!algorithm->IsBusy() && !store->IsAborted()) {
                        store->incCalculationStep();
                        QString finishedM = mPlugins[2]->Name();
                        finishedM.append(" - plugin started");
                        CLogController::instance().frameworkMessage(finishedM);
                        algorithm->run(store, [this](CContextDataStore *store) {
                            IAlgorithm* algorithm = mPlugins[3]->getAlgorithm();
                            algorithm->setLogger(&CLogController::instance());
                            if (!algorithm->IsBusy() && !store->IsAborted()) {
                                store->incCalculationStep();
                                QString finishedM = mPlugins[3]->Name();
                                finishedM.append(" - plugin started");
                                CLogController::instance().frameworkMessage(finishedM);
                                algorithm->run(store, [this](CContextDataStore *store) {
                                    sigDataStoreFinished(store);
                                });
                            } else { sigDataStoreFinished(store);

                            }
                        });
                    } else { sigDataStoreFinished(store);

                    }
                });
            } else { sigDataStoreFinished(store);

            }
        });
    } else { sigDataStoreFinished(store);

    };
#endif
}

bool CFourPhaseWorkflow::checkAvailableDataTypes() const {
    QStringList dataTypes;
    dataTypes.push_back(DT_INPUTIMAGES);

    for (uint i = 0; i < getStepCount(); i++) {
        IPlugin *plugin = mPlugins[i];

        if (plugin == nullptr) {
            continue;
        }

        auto input = plugin->getAlgorithm()->getInputDataTypes();
        auto output = plugin->getAlgorithm()->getOutputDataTypes();

        for (QString type : input) {
            if (!dataTypes.contains(type)) {
                return false;
            }
        }

        for (QString type : output) {
            dataTypes.push_back(type);
        }
    }

    return true;
}
