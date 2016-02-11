#include "cfourphaseworkflow.h"
#include "workflow/plugin/cpluginmanager.h"


CFourPhaseWorkflow::CFourPhaseWorkflow() {
    mPlugins = new IPlugin*[getStepCount()];
    mDataStores = new QList<CFourPhaseDataStore*>();
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
    return *reinterpret_cast<QList<AContextDataStore *> *>(mDataStores);
}

AContextDataStore* CFourPhaseWorkflow::addDataStore() {
    auto dataStore = new CFourPhaseDataStore();
    mDataStores->append(dataStore);
    return reinterpret_cast<AContextDataStore *>(dataStore);
}

bool CFourPhaseWorkflow::removeDataStore(QString id) {
    CFourPhaseDataStore *result = FindStore(id);

    if (result != nullptr) {
        mDataStores->removeAll(result);
        delete result;
        return true;
    } else {
        return false;
    }
}

bool CFourPhaseWorkflow::run(const QString storeId) {
    if (!checkAvailableDataTypes()) {
        return false;
    }

    __PREPARE_ALGORITHM(FindStore(storeId));

    if (store == nullptr) {
        return false;
    }

    if (mPlugins[0]->getAlgorithm()->IsBusy()) {
        return false;
    }

    __RUN_ALGORITHM(mPlugins[0],
        __RUN_ALGORITHM(mPlugins[1],
            __RUN_ALGORITHM(mPlugins[2],
                __RUN_ALGORITHM(mPlugins[3], ))));
}

qint32 CFourPhaseWorkflow::getState(const QString storeId) const {
    auto store = FindStore(storeId);

    if(store == nullptr) {
        return -1;
    }

    return store->getCurrentCalculationStep();
}

void CFourPhaseWorkflow::stop(const QString storeId) {
    auto store = FindStore(storeId);

    if (store != nullptr) {
        store->SetIsAborted(true);
    }
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


CFourPhaseDataStore *CFourPhaseWorkflow::FindStore(QString id) const {
    for (auto store : *mDataStores) {
        if (store->getId() == id) {
            return store;
        }
    }

    return nullptr;
}