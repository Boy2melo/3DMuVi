#include "plugin.h"
#include "workflow/plugin/cpluginmanager.h"
#include <QPluginLoader>
#include <QJsonObject>

//----------------------------------------------
// Autofunctions, no adjustments needed
//----------------------------------------------
QString TestPlugin::Autor() const {
    return "QTest";
}

qint32 TestPlugin::Version() const {
    return 1;
}

QDate TestPlugin::Date() const {
    return QDate::fromString("2016-03-10", "yyyy-MM-dd");
}

QString TestPlugin::Name() const {
    return mName;
}

QJsonObject TestPlugin::GetParameterJson() const {
    return mParameters.value("Default").toObject();
}

QJsonObject TestPlugin::GetParameterDescriptionJson() const {
    return mParameters.value("Description").toObject();
}

IAlgorithm *TestPlugin::getAlgorithm() const {
    return static_cast<IAlgorithm*>(mAlgorithm);
}

TestAlgorithm* TestPlugin::getAlgorithmCast() const {
    return mAlgorithm;
}

TestPlugin ::~TestPlugin () {
    delete mAlgorithm;
}

bool TestPlugin::ValidateParameters(QJsonObject *params) const {
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

QString TestPlugin ::GetPluginType() const {
    return mType;
}

void TestPlugin::Initialize(QPluginLoader* loader) {
    // do nothing, will never happen
}

TestPlugin::TestPlugin(QString type, QString name, QStringList inputTypes, QStringList outputTypes){
    mType = type;
    mName = name;
    mAlgorithm = new TestAlgorithm(inputTypes, outputTypes);
}
