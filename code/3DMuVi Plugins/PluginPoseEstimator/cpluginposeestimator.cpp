#include "cpluginposeestimator.h"

#include <QJsonDocument>

#include "workflow/workflow/ccontextdatastore.h"

//----------------------------------------------
// Autofunctions, no adjustments needed
//----------------------------------------------

bool CPluginPoseEstimator::IsBusy() const{
    return mIsBusy;
}

void CPluginPoseEstimator::setLogger(CLogController *controller) {
    mLogger = controller;
}

void CPluginPoseEstimator::setParameters(QJsonObject settings){
    mSettings = settings;
}

CPluginPoseEstimator::CPluginPoseEstimator() {
    mIsBusy = false;
    mLogger = nullptr;
}

bool CPluginPoseEstimator::run(){
    if(!IsBusy() && mLogger != nullptr && validateParameters(mSettings)){
        executeAlgorithm();
        return true;
    }
    return false;
}

std::shared_ptr<IPoseEstimator> CPluginPoseEstimator::newPoseEstimator() {
    return std::shared_ptr<IPoseEstimator>(new CPluginPoseEstimator);
}

IPoseEstimator::Factory* newPoseEstimator = &CPluginPoseEstimator::newPoseEstimator;
