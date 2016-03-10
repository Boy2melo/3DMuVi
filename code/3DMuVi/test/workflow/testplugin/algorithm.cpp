#include "algorithm.h"
#include "workflow/workflow/ccontextdatastore.h"

//----------------------------------------------
// Autofunctions, no adjustments needed
//----------------------------------------------

bool TestAlgorithm::IsBusy() const {
    return mIsBusy;
}

void TestAlgorithm::setLogger(CLogController *controller) {
    mLogger = controller;
}

void TestAlgorithm::setParameters(QJsonObject *settings) {
    mSettings = settings;
}

TestAlgorithm::TestAlgorithm(QStringList inputTypes, QStringList outputTypes) {
    mIsBusy = false;
    mSettings = nullptr;
    mLogger = nullptr;

    mInputTypes.append(inputTypes);
    mOutputTypes.append(outputTypes);
}

QStringList TestAlgorithm::getInputDataTypes() const {
    return mInputTypes;
}

QStringList TestAlgorithm::getOutputDataTypes() const {
    return mOutputTypes;
}

void TestAlgorithm::setValidateCallback(std::function<bool(const QJsonObject*)> validateFunc) {
    mValidateFunc = validateFunc;
}

void TestAlgorithm::run(CContextDataStore *dataStore, std::function<void(CContextDataStore *)> callback) {
    if (!IsBusy() && ValidateParameters(mSettings)) {
        executeAlgorithm(dataStore);
        callback(dataStore);
    }
}

void TestAlgorithm::setExecCallback(std::function<void(CContextDataStore*)> execFunc) {
    mExecFunc = execFunc;
}

bool TestAlgorithm::ValidateParameters(const QJsonObject *params) {
    // First level typechecks are already done, see plugin.cpp
    if (mValidateFunc)
        return mValidateFunc(params);
    return true;
}

void TestAlgorithm::executeAlgorithm(CContextDataStore *store) {
    if (mExecFunc)
        mExecFunc(store);
}