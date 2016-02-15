#include "examplealgorithm.h"
#include "workflow/workflow/ccontextdatastore.h"
//----------------------------------------------
// Autofunctions, no adjustments needed
//----------------------------------------------

bool ExampleAlgorithm::IsBusy() const{
    return mIsBusy;
}

void ExampleAlgorithm::setLogger(CLogController *controller) {
    mLogger = controller;
}

void ExampleAlgorithm::setParameters(CAlgorithmSettingController *settings){
    mSettings = settings;
}

//----------------------------------------------------------
// ToDo-Functions
//----------------------------------------------------------
ExampleAlgorithm::ExampleAlgorithm() {
    mIsBusy = false;
    mSettings = nullptr;
    mLogger = nullptr;
}

QStringList ExampleAlgorithm::getInputDataTypes() const {
    // TODO: set needed data package types
    QStringList dataTypes;
    dataTypes.push_back(DT_DEPTH);

    return dataTypes;
}

QStringList ExampleAlgorithm::getOutputDataTypes() const {
    // TODO: set produced data package types
    QStringList dataTypes;
    dataTypes.push_back(DT_DEPTH);

    return dataTypes;
}

void ExampleAlgorithm::run(CContextDataStore *dataStore, std::function<void (CContextDataStore *)> callback){
    if(IsBusy() || mSettings == nullptr || mLogger == nullptr){
        return;
    }

    // Needs to be called when all calculations are done
    callback(dataStore);
}
