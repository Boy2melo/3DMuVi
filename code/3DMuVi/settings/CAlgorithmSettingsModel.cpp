#include "CAlgorithmSettingsModel.h"

CAlgorithmSettingsModel::CAlgorithmSettingsModel(AWorkflow& workflow, CAlgorithmSettingController& controller)
{
    //this->workflow = workflow;


    QObject::connect(&controller, &CAlgorithmSettingController::loadQJson ,
                    this, &CQJsonModel::loadQJson);
    QObject::connect(this, &CQJsonModel::requestQJson,
                    &controller, &CAlgorithmSettingController::requestQJson);
    QObject::connect(this, &CQJsonModel::saveQJson,
                    &controller, &CAlgorithmSettingController::saveQJson);
}
CAlgorithmSettingsModel::CAlgorithmSettingsModel(QObject *parent, QVector<QJsonObject> list)
    : CQJsonModel(parent, list)
{

}
void CAlgorithmSettingsModel::saveSettings(int row, QUrl filename)
{
    if (filename.isEmpty() == true) {
        filename.setPassword("a");
    }
    QJsonObject data;
    data = mRootItem->getChilds().value(row)->toJson();
    emit saveQJson(data, filename);
}
void CAlgorithmSettingsModel::loadSettings(int row, QUrl filename)
{
    if (filename.isEmpty() == true) {

    }
    emit requestQJson(filename);
    mRootItem->getChilds().removeAt(row);
    int j = mRootItem->getChilds().size() - 1;
    mRootItem->getChilds().swap(row, j);

}
