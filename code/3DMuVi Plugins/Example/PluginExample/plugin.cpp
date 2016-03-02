#include "plugin.h"
#include "workflow/plugin/cpluginmanager.h"
#include <QPluginLoader>
#include <QJsonObject>

//----------------------------------------------
// Autofunctions, no adjustments needed
//----------------------------------------------
QString CLASS_GEN(Plugin)::Autor() const {
    return PLUGIN_AUTHOR;
}

qint32 CLASS_GEN(Plugin)::Version() const {
    return PLUGIN_VERSION;
}

QDate CLASS_GEN(Plugin)::Date() const {
    return QDate::fromString(PLUGIN_DATE, "yyyy-MM-dd");
}

QString CLASS_GEN(Plugin)::Name() const {
    return STRINGIFY(PLUGIN_NAME);
}

QJsonObject CLASS_GEN(Plugin)::GetParameterJson() const {
    return mParameters.value("Default").toObject();
}

QJsonObject CLASS_GEN(Plugin)::GetParameterDescriptionJson() const {
    return mParameters.value("Description").toObject();
}

IAlgorithm *CLASS_GEN(Plugin)::getAlgorithm() const {
    return static_cast<IAlgorithm*>(mAlgorithm);
}

CLASS_GEN(Plugin)::~CLASS_GEN(Plugin)() {
    delete mAlgorithm;
}

bool CLASS_GEN(Plugin)::ValidateParameters(QJsonObject *params) const {
    // First level type check over given parameters
    for(QJsonObject::iterator itr = params->begin(); itr != params->end(); itr++){
        // Get reference type
        auto ref = GetParameterJson().value(itr.key());

        if(ref.type() != itr.value().type()) {
            return false;
        }
    }

    return mAlgorithm->ValidateParameters(params);
}

QString CLASS_GEN(Plugin)::GetPluginType() const {
    return _PLUGIN_TYPE;
}

void CLASS_GEN(Plugin)::Initialize(QPluginLoader* loader) {
    mParameters = loader->metaData().value("MetaData").toObject();
}

CLASS_GEN(Plugin)::CLASS_GEN(Plugin)(){
    mAlgorithm = new CLASS_GEN(Algorithm)();
}
