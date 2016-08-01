#include "pluginsurf.h"

#include <QJsonDocument>

//----------------------------------------------
// Autofunctions, no adjustments needed
//----------------------------------------------

bool PluginSurf::IsBusy() const{
    return mIsBusy;
}

void PluginSurf::setLogger(CLogController *controller) {
    mLogger = controller;
}

void PluginSurf::setParameters(QJsonObject settings){
    mSettings = settings;
}

PluginSurf::PluginSurf() {
    mIsBusy = false;
    mLogger = nullptr;
}

PluginSurf::~PluginSurf() {

}

bool PluginSurf::run(){
    if(!IsBusy() && mLogger != nullptr && validateParameters(mSettings)){
        executeAlgorithm();
        return true;
    }
    return false;
}

std::shared_ptr<IFeatureMatcher> PluginSurf::newFeatureMatcher()
{
  return std::shared_ptr<IFeatureMatcher>(new PluginSurf);
}

IFeatureMatcher::Factory* newFeatureMatcher = &PluginSurf::newFeatureMatcher;
