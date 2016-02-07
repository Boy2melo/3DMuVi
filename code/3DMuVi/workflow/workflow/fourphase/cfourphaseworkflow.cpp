#include "cfourphaseworkflow.h"
#include "workflow/plugin/cpluginmanager.h"


CFourPhaseWorkflow::CFourPhaseWorkflow() {
    mPlugins = new IPlugin*[getStepCount()];
}


quint32 CFourPhaseWorkflow::getStepCount() const {
    return 4;
}

QString CFourPhaseWorkflow::getAlgorithmType(const quint32 step) const {
    switch (step) {
    case 0:
        return CPluginManager::PT_FeatureMatcher;
    case 1:
        return CPluginManager::PT_DepthMapper;
    case 2:
        return CPluginManager::PT_PoseEstimator;
    case 3:
        return CPluginManager::PT_PclReconstructor;
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

QList<AContextDataStore*> CFourPhaseWorkflow::getDataStores() const {
    return QList<AContextDataStore *>();
}

AContextDataStore* CFourPhaseWorkflow::addDataStore() {
    return nullptr;
}

bool CFourPhaseWorkflow::removeDataStore(QString id) {
    return false;
}

void CFourPhaseWorkflow::run(const QString storeId) {}

quint32 CFourPhaseWorkflow::getState(const QString storeId) const {
    return 0;
}

void CFourPhaseWorkflow::stop(const QString storeId) {}

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

        foreach(QString type, input) {
            if (!dataTypes.contains(type)) {
                return false;
            }
        }

        foreach(QString type, output) {
            dataTypes.push_back(type);
        }
    }

    return true;
}
