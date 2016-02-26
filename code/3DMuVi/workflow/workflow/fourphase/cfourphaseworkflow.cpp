#include "cfourphaseworkflow.h"
#include "workflow/plugin/cpluginmanager.h"


CFourPhaseWorkflow::CFourPhaseWorkflow() : AWorkflow() {
    mPlugins = new IPlugin*[getStepCount()];
}


quint32 CFourPhaseWorkflow::getStepCount() const {
    return 4;
}

QString CFourPhaseWorkflow::getAlgorithmType(const quint32 step) const {
    switch (step) {
    case 0:
        return PT_FeatureMatcher;
    case 1:
        return PT_DepthEstimator;
    case 2:
        return PT_PoseEstimator;
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
    __RUN_ALGORITHM(mPlugins[0],
        __RUN_ALGORITHM(mPlugins[1],
            __RUN_ALGORITHM(mPlugins[2],
                __RUN_ALGORITHM(mPlugins[3], emit sigDataStoreFinished(store); ))));
}

bool CFourPhaseWorkflow::checkAvailableDataTypes() const {
    QStringList dataTypes;
    //TODO add input images as initial item

    for (int i = 0; i < getStepCount(); i++) {
        IPlugin *plugin = mPlugins[i];

        if (plugin == nullptr) {
            continue;
        }

        auto input = plugin->getAlgorithm()->getInputDataTypes();
        auto output = plugin->getAlgorithm()->getOutputDataTypes();

        for(QString type : input) {
            if (!dataTypes.contains(type)) {
                return false;
            }
        }

        for(QString type : output) {
            dataTypes.push_back(type);
        }
    }

    return true;
}