#include "plugin.h"
#include "workflow/plugin/cpluginmanager.h"
#include <QPluginLoader>
#include <QJsonObject>

//----------------------------------------------
// Autofunctions, no adjustments needed
//----------------------------------------------
QString _CLASS_GEN(Plugin)::Autor() const {
    return str(_PLUGIN_AUTHOR);
}

qint32 _CLASS_GEN(Plugin)::Version() const {
    return _PLUGIN_VERSION;
}

QDate _CLASS_GEN(Plugin)::Date() const {
    return QDate::fromString(str(_PLUGIN_DATE), "yyyy-MM-dd");
}

QJsonObject _CLASS_GEN(Plugin)::GetParameterJson() const {
    return mParameters.value("Default").toObject();
}

QJsonObject _CLASS_GEN(Plugin)::GetParameterDescriptionJson() const {
    return mParameters.value("Description").toObject();
}

IAlgorithm *_CLASS_GEN(Plugin)::getAlgorithm() const {
    return static_cast<IAlgorithm*>(mAlgorithm);
}

_CLASS_GEN(Plugin)::~_CLASS_GEN(Plugin)() {
    delete mAlgorithm;
}

bool _CLASS_GEN(Plugin)::ValidateParameters(QJsonObject *params) const {
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

QString _CLASS_GEN(Plugin)::GetPluginType() const {
    return _PLUGIN_TYPE;
}

void _CLASS_GEN(Plugin)::Initialize(QPluginLoader* loader) {
    mParameters = loader->metaData().value("MetaData").toObject();
}

_CLASS_GEN(Plugin)::_CLASS_GEN(Plugin)(){
    mAlgorithm = new _CLASS_GEN(Algorithm)();
}
