#include "CAlgorithmSettingsModel.h"

CAlgorithmSettingsModel::CAlgorithmSettingsModel(CPluginManager& manager)
{

}
CAlgorithmSettingsModel::CAlgorithmSettingsModel(QObject *parent, QVector<QJsonObject> list)
    : CQJsonModel(parent, list)
{

}
