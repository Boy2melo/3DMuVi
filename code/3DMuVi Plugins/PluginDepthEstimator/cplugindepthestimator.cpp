#include "cplugindepthestimator.h"

#include <QJsonDocument>

#include "workflow/workflow/ccontextdatastore.h"

//----------------------------------------------
// Autofunctions, no adjustments needed
//----------------------------------------------

bool CPluginDepthEstimator::IsBusy() const{
    return mIsBusy;
}

void CPluginDepthEstimator::setLogger(CLogController *controller) {
    mLogger = controller;
}

void CPluginDepthEstimator::setParameters(QJsonObject settings){
    mSettings = settings;
}

CPluginDepthEstimator::CPluginDepthEstimator() {
    mIsBusy = false;
    mLogger = nullptr;
}

bool CPluginDepthEstimator::run(){
    if(!IsBusy() && mLogger != nullptr && validateParameters(mSettings)){
        executeAlgorithm();
        return true;
    }
    return false;
}

std::shared_ptr<IDepthEstimator> CPluginDepthEstimator::newDepthEstimator() {
    return std::shared_ptr<IDepthEstimator>(new CPluginDepthEstimator);
}

IDepthEstimator::Factory* newDepthEstimator = &CPluginDepthEstimator::newDepthEstimator;
