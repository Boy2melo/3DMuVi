#include "aworkflow.h"
#include <QTimer>

AWorkflow::AWorkflow() {
    mDataStores = new QList<CContextDataStore*>();
}

bool AWorkflow::run(const QString storeId) {
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

    // Start a new Thread
    auto *thread = new QThread(this);
    auto *timer = new QTimer(nullptr);
    thread->start();
    timer->setInterval(10);
    timer->setSingleShot(true);
    timer->moveToThread(thread);
    
    //TODO: Timer not working
    //connect(timer, &QTimer::timeout, [this, store]() {
        this->executeAlgorithm(store);
    //});
    //connect(thread, &QThread::started, timer, &QTimer::start);
    connect(thread, &QThread::finished, thread, &QThread::deleteLater);     // Prevent memory leak
    connect(thread, &QThread::finished, timer, &QTimer::deleteLater);
    
    timer->start(10);
    
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
