#include "surfplugin.h"
#include "workflow/plugin/cpluginmanager.h"
#include <QPluginLoader>
#include <QJsonObject>

//----------------------------------------------
// Autofunctions, no adjustments needed
//----------------------------------------------
QString SurfPlugin::Autor() const {
    return mAutor;
}

qint32 SurfPlugin::Version() const {
    return mVersion;
}

QDate SurfPlugin::Date() const {
    return mDate;
}

QJsonObject SurfPlugin::GetParameterJson() const {
    return mParameters;
}

IAlgorithm *SurfPlugin::getAlgorithm() const {
    return static_cast<IAlgorithm*>(mAlgorithm);
}

SurfPlugin::~SurfPlugin() {
    delete mAlgorithm;
}

//----------------------------------------------------------
// ToDo-Functions
//----------------------------------------------------------

bool SurfPlugin::ValidateParameters(QJsonObject *) const {
    // TODO: Check parameters
    return true;
}

QString SurfPlugin::GetPluginType() const {
    return PT_FeatureMatcher;
}

SurfPlugin::SurfPlugin(){
    QPluginLoader *pluginLoader = new QPluginLoader(BUILD_NAME);
    QJsonObject metadata = pluginLoader->metaData().value("MetaData").toObject();
    mAutor = metadata.value("Autor").toString();
    mVersion = metadata.value("Version").toInt();
    mDate = QDate::fromString(metadata.value("Date").toString(), "yyyy-MM-dd");
    mParameters = metadata.value("Parameters").toObject();
    delete pluginLoader;

    //----------------
    // Own Constructor
    // TODO: Initialize Algorithm to heap
    //----------------
    mAlgorithm = new SurfAlgorithm();
}
