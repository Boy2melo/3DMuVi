#include "aworkflow.h"


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
}
