#include "workflow/workflow/ccontextdatastore.h"
#include "algorithm.h"

//----------------------------------------------
// Autofunctions, no adjustments needed
//----------------------------------------------

bool CLASS_GEN(Algorithm)::IsBusy() const{
    return mIsBusy;
}

void CLASS_GEN(Algorithm)::setLogger(CLogController *controller) {
    mLogger = controller;
}

void CLASS_GEN(Algorithm)::setParameters(QJsonObject *settings){
    mSettings = settings;
}

CLASS_GEN(Algorithm)::CLASS_GEN(Algorithm)() {
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
        mIsBusy = true;
        executeAlgorithm(dataStore);
        callback(dataStore);
        mIsBusy = false;
    }
}
