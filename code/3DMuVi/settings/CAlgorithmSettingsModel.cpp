#include "CAlgorithmSettingsModel.h"

CAlgorithmSettingsModel::CAlgorithmSettingsModel(AWorkflow& workflow, CAlgorithmSettingController& controller)
{
    QObject::connect(&controller, &CAlgorithmSettingController::loadQJson ,
                    this, &CQJsonModel::loadQJson);
    QObject::connect(this, &CQJsonModel::requestQJson,
                    &controller, &CAlgorithmSettingController::requestQJson);
    QObject::connect(this, &CQJsonModel::saveQJson,
                    &controller, &CAlgorithmSettingController::saveQJson);
    this->workflow = &workflow;
    this->settingcontroller = &controller;
    /*for (quint32 i = 0; i < workflow.getStepCount(); i++) {
    QJsonObject object = workflow.getStep(i)->GetParameterJson();
        this->loadQJson(object);
        QUrl url = QUrl();
        url.setPassword("a" + object.keys().value(0));
        emit saveQJson(object, url);
    }*/


}
CAlgorithmSettingsModel::CAlgorithmSettingsModel(QObject *parent, QVector<QJsonObject> list)
    : CQJsonModel(parent, list)
{

}
void CAlgorithmSettingsModel::saveSettings(int row, QUrl filename)
{
    QJsonObject data;
    data = mRootItem->getChilds().value(row)->toJson();
    if (filename.isEmpty() == true) {
        filename.setPassword("a" + workflow->getStep(row)->GetPluginType());
    }
    if (workflow->getStep(row)->ValidateParameters(&data)) {
        emit saveQJson(data, filename);
        workflow->getStep(row)->getAlgorithm()->setParameters(&data);
    }
}
void CAlgorithmSettingsModel::loadSettings(int row, QUrl filename)
{
    if (filename.isEmpty() == true) {
        filename.setPassword("a" + workflow->getStep(row)->GetPluginType());
    }
    emit requestQJson(filename);
    mRootItem->getChilds().removeAt(row);
    int j = mRootItem->getChilds().size() - 1;
    mRootItem->getChilds().swap(row, j);

}
void CAlgorithmSettingsModel::algorithmChanged(int step)
{
    QJsonObject object = workflow->getStep(step)->GetParameterJson();
    loadQJson(object);
    mRootItem->getChilds().swap(step, mRootItem->getChilds().size() - 1);
    mRootItem->getChilds().removeLast();
    QUrl url = QUrl();
    url.setPassword("a" + object.keys().value(0));
    emit saveQJson(object, url);
}
