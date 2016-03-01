#include "algorithm.h"
#include "workflow/workflow/ccontextdatastore.h"

//----------------------------------------------
// Autofunctions, no adjustments needed
//----------------------------------------------

bool _CLASS_GEN(Algorithm)::IsBusy() const{
    return mIsBusy;
}

void _CLASS_GEN(Algorithm)::setLogger(CLogController *controller) {
    mLogger = controller;
}

void _CLASS_GEN(Algorithm)::setParameters(QJsonObject *settings){
    mSettings = settings;
}

_CLASS_GEN(Algorithm)::_CLASS_GEN(Algorithm)() {
    OnInitialize();
}

QStringList _CLASS_GEN(Algorithm)::getInputDataTypes() const {
    return mInputTypes;
}

QStringList _CLASS_GEN(Algorithm)::getOutputDataTypes() const {
    return mOutputTypes;
}

void _CLASS_GEN(Algorithm)::run(CContextDataStore *dataStore, std::function<void (CContextDataStore *)> callback){
    if(!IsBusy() && mSettings != nullptr && mLogger != nullptr && ValidateParameters(mSettings)){
        mIsBusy = true;
        executeAlgorithm(dataStore);
        callback(dataStore);
        mIsBusy = false;
    }
}
