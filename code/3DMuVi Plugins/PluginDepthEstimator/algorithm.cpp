#include "algorithm.h"
#include "workflow/workflow/ccontextdatastore.h"

//----------------------------------------------
// Autofunctions, no adjustments needed
//----------------------------------------------

bool CLASS_GEN(Algorithm)::IsBusy() const{
    return mIsBusy;
}

void CLASS_GEN(Algorithm)::setLogger(CLogController *controller) {
    mLogger = controller;
}

void CLASS_GEN(Algorithm)::setParameters(QJsonObject settings){
    mQJson = settings;
    mSettings = &mQJson;
}

CLASS_GEN(Algorithm)::CLASS_GEN(Algorithm)() {
    mIsBusy = false;
    mSettings = nullptr;
    mLogger = nullptr;

    OnInitialize();
}

QStringList CLASS_GEN(Algorithm)::getInputDataTypes() const {
    return mInputTypes;
}

QStringList CLASS_GEN(Algorithm)::getOutputDataTypes() const {
    return mOutputTypes;
}

void CLASS_GEN(Algorithm)::run(CContextDataStore *dataStore, std::function<void (CContextDataStore *)> callback){
    if(!IsBusy() && mSettings != nullptr && mLogger != nullptr && ValidateParameters(mSettings)){
        executeAlgorithm(dataStore);
        callback(dataStore);
    }
}
