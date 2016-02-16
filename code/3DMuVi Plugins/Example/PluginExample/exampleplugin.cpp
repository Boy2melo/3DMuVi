#include "exampleplugin.h"
#include "workflow/plugin/cpluginmanager.h"
#include <QPluginLoader>
#include <QJsonObject>

//----------------------------------------------
// Autofunctions, no adjustments needed
//----------------------------------------------
QString ExamplePlugin::Autor() const {
    return mAutor;
}

qint32 ExamplePlugin::Version() const {
    return mVersion;
}

QDate ExamplePlugin::Date() const {
    return mDate;
}

QJsonObject ExamplePlugin::GetParameterJson() const {
    return mParameters;
}

IAlgorithm *ExamplePlugin::getAlgorithm() const {
    return static_cast<IAlgorithm*>(mAlgorithm);
}

ExamplePlugin::~ExamplePlugin() {
    delete mAlgorithm;
}

//----------------------------------------------------------
// ToDo-Functions
//----------------------------------------------------------

bool ExamplePlugin::ValidateParameters(QJsonObject *) const {
    // TODO: Check parameters
    return true;
}

QString ExamplePlugin::GetPluginType() const {
    // TODO: return type as defined in CPluginManager
    return PT_PoseEstimator;
}

ExamplePlugin::ExamplePlugin(){
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
    mAlgorithm = new ExampleAlgorithm();
}
