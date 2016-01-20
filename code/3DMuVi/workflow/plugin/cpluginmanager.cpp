#include "cpluginmanager.h"

CPluginManager* CPluginManager::mInstance = NULL;

CPluginManager::CPluginManager() {
}

CPluginManager* CPluginManager::Instance() {
    if(mInstance == NULL){
        mInstance = new CPluginManager();
    }

    return mInstance;
}

QVector<APlugin*> CPluginManager::getPlugins(){
    return mPlugins;
}
