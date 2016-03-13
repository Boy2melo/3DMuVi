#include "aworkflow.h"
#include <QTimer>

AWorkflow::~AWorkflow() {
    //TODO: datastores in shared_ptr
    for(auto store : *mDataStores) {
        delete store;
    }
    delete mDataStores;
}

AWorkflow::AWorkflow() {
    mDataStores = new QList<CContextDataStore*>();
}

bool AWorkflow::run(const QString storeId, bool multiThread) {
    mMutex.lock();      // Only one thread can do a check, so a data store is guaranteed to only be executed once

    // do a basic check
    auto store = FindStore(storeId);

    if (!checkAvailableDataTypes()) {
        mMutex.unlock();
        return false;   //Workflow wrong configured
    }

    if (store == nullptr) {
        mMutex.unlock();
        return false;   // Invalid store
    }

    if (mRunningAlgorithms.contains(store->getId())) {
        mMutex.unlock();
        return false;   // Store already in use
    }

    store->SetIsAborted(false);
    store->resetCalculationStep();

    if (multiThread) {
        // Start a new Thread
        auto *thread = new QThread(this);

        this->moveToThread(thread);
        QTimer::singleShot(0, this, [=] {
            this->executeAlgorithm(store);
        });

        connect(thread, &QThread::finished, thread, &QThread::deleteLater);
        thread->start();
    }else {
        this->executeAlgorithm(store);
    }

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
        result = nullptr;
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
