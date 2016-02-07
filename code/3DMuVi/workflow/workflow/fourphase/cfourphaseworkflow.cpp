#include "cfourphaseworkflow.h"
#include "workflow/plugin/cpluginmanager.h"

IPlugin* CFourPhaseWorkflow::mPlugins[4] = { nullptr, nullptr, nullptr, nullptr };
QList<CFourPhaseDataStore *> CFourPhaseWorkflow::mDataStores;

CFourPhaseWorkflow::CFourPhaseWorkflow() {}


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

QVector<AContextDataStore*> CFourPhaseWorkflow::getDataStores() const {}

AContextDataStore* CFourPhaseWorkflow::addDataStore() {}

bool CFourPhaseWorkflow::removeDataStore(QString id) {}

void CFourPhaseWorkflow::run(const QString storeId) {}

quint32 CFourPhaseWorkflow::getState(const QString storeId) const {}

void CFourPhaseWorkflow::stop(const QString storeId) {}

bool CFourPhaseWorkflow::checkAvailableDataTypes() const {
    QStringList dataTypes;
    //TODO add input images as initial item

    foreach(IPlugin *plugin, mPlugins) {
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
