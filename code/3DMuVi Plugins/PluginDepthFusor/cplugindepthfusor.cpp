#include "cplugindepthfusor.h"

#include <QJsonDocument>

#include "workflow/workflow/ccontextdatastore.h"

//----------------------------------------------
// Autofunctions, no adjustments needed
//----------------------------------------------

bool CPluginDepthFusor::IsBusy() const{
    return mIsBusy;
}

void CPluginDepthFusor::setLogger(CLogController *controller) {
    mLogger = controller;
}

void CPluginDepthFusor::setParameters(QJsonObject settings){
    mSettings = settings;
}

CPluginDepthFusor::CPluginDepthFusor() {
    mIsBusy = false;
    mLogger = nullptr;
}

bool CPluginDepthFusor::run(){
    if(!IsBusy() && mLogger != nullptr && validateParameters(mSettings)){
        executeAlgorithm();
        return true;
    }
    return false;
}

std::shared_ptr<IFusor> CPluginDepthFusor::newFusor() {
    return std::shared_ptr<IFusor>(new CPluginDepthFusor);
}

IFusor::Factory* newFusor = &CPluginDepthFusor::newFusor;
