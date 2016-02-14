#include "aworkflow.h"


AWorkflow::AWorkflow() {
    mDataStores = new QList<CContextDataStore*>();
}

bool AWorkflow::run(const QString storeId) {
    mMutex.lock();      // Only one thread can do a check, so a data store is guaranteed to only be executed once
    
    // do a basic check
    auto store = FindStore(storeId);

    if(!checkAvailableDataTypes()) {
        return false;   //Workflow wrong configured
    }

    if(store == nullptr) {
        return false;   // Invalid store
    }

    if(mRunningAlgorithms.contains(store->getId())) {
        return false;   // Store already in use
    }

    store->SetIsAborted(false);
    store->resetCalculationStep();
    
    // Start a new Thread
    QThread *thread = new QThread();
    QThread *currentThread = this->thread();

    connect(this, &AWorkflow::sigExecuteAlgorithm, this, &AWorkflow::executeAlgorithm);
    connect(thread, &QThread::finished, thread, &QThread::deleteLater);     // Prevent memory leak

    this->moveToThread(thread);

    emit sigExecuteAlgorithm(store);

    this->moveToThread(currentThread);

    disconnect(this, &AWorkflow::sigExecuteAlgorithm, this, &AWorkflow::executeAlgorithm);

    mMutex.unlock();

    return true;
}


QList<CContextDataStore*> AWorkflow::getDataStores() const {
    return *mDataStores;
}

CContextDataStore* AWorkflow::addDataStore() const {
    auto dataStore = new CContextDataStore();
    mDataStores->append(dataStore);
    return dataStore;
}

bool AWorkflow::removeDataStore(QString id) const {
    auto *result = FindStore(id);

    if (result != nullptr) {
        mDataStores->removeAll(result);
        delete result;
        return true;
    } else {
        return false;
    }
}

qint32 AWorkflow::getState(const QString storeId) const {
    auto store = FindStore(storeId);

    if (store == nullptr) {
        return -1;
    }

    return store->getCurrentCalculationStep();
}

void AWorkflow::stop(const QString storeId) const {
    auto store = FindStore(storeId);

    if (store != nullptr) {
        store->SetIsAborted(true);
    }
}


CContextDataStore* AWorkflow::FindStore(QString id) const {
    for (auto store : *mDataStores) {
        if (store->getId() == id) {
            return static_cast<CContextDataStore *>(store);
        }
    }

    return nullptr;
}
