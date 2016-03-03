#include "CAlgorithmSettingsModel.h"

CAlgorithmSettingsModel::CAlgorithmSettingsModel(AWorkflow& workflow, CAlgorithmSettingController& controller) {
    connect(&controller, &CAlgorithmSettingController::loadQJson,
            this, &CQJsonModel::loadQJson);
    connect(this, &CQJsonModel::requestQJson,
            &controller, &CAlgorithmSettingController::requestQJson);
    connect(this, &CQJsonModel::saveQJson,
            &controller, &CAlgorithmSettingController::saveQJson);
    this->workflow = &workflow;
    this->settingcontroller = &controller;
}

CAlgorithmSettingsModel::CAlgorithmSettingsModel(QObject* parent, QVector<QJsonObject> list)
    : CQJsonModel(parent, list), workflow(nullptr), settingcontroller(nullptr) {

}

void CAlgorithmSettingsModel::saveSettings(int row, QUrl filename) {
    QJsonObject data;
    CQJsonTreeItem* parentItem = mRootItem->getChilds().value(row);

    for(CQJsonTreeItem* i : parentItem->getChilds())
    {
      QJsonObject o = i->toJson();
      QString key = o.keys().at(0);
      data.insert(key, o.take(key));
    }
    if (filename.isEmpty() == true) {
        filename.setPassword("a" + workflow->getStep(row)->Name());
    }
    if (workflow->getStep(row)->ValidateParameters(&data)) {
        emit saveQJson(data, filename);
        workflow->getStep(row)->getAlgorithm()->setParameters(&data);
    }
}

void CAlgorithmSettingsModel::loadSettings(int row, QUrl filename) {
    if (filename.isEmpty() == true) {
        filename.setPassword("a" + workflow->getStep(row)->Name());
    }
    emit requestQJson(filename);
    int j = mRootItem->getChilds().size();
    mRootItem->getChilds().swap(row, j);
    mRootItem->getChilds().removeAt(j);
}

void CAlgorithmSettingsModel::algorithmChanged(int step) {
    QJsonObject object = workflow->getStep(step)->GetParameterJson();
    loadQJson(object);
    mRootItem->getChilds().swap(step, mRootItem->getChilds().size() - 1);
    mRootItem->getChilds().removeLast();
    QUrl url;
    url.setPassword("a" + object.keys().value(0));
    emit saveQJson(object, url);
}

bool CAlgorithmSettingsModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
  if(CQJsonModel::setData(index, value,role))
  {
    CQJsonTreeItem* parentItem = backtrack(index);
    QJsonObject params;
    IPlugin* plugin = workflow->getStep(parentItem->row());

    for(CQJsonTreeItem* i : parentItem->getChilds())
    {
      QJsonObject o = i->toJson();
      QString key = o.keys().at(0);
      params.insert(key, o.take(key));
    }

    if(plugin->ValidateParameters(&params))
    {
      plugin->getAlgorithm()->setParameters(new QJsonObject(params));
    }
    return true;
  }
  return false;
}

