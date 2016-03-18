#include "CAlgorithmSettingsModel.h"
#include "logger/controll/CLogController.h"

CAlgorithmSettingsModel::CAlgorithmSettingsModel(AWorkflow& workflow, CAlgorithmSettingController& controller) {
    connect(&controller, &CAlgorithmSettingController::loadQJson,
            this, &CQJsonModel::loadQJson);
    connect(this, &CQJsonModel::requestQJson,
            &controller, &CAlgorithmSettingController::requestQJson);
    connect(this, &CQJsonModel::saveQJson,
            &controller, &CAlgorithmSettingController::saveQJson);
    connect(this, &CQJsonModel::saveQJsonEx,
            &controller, &CAlgorithmSettingController::saveQJsonEx);
    this->workflow = &workflow;
    this->settingcontroller = &controller;
}

CAlgorithmSettingsModel::CAlgorithmSettingsModel(QObject* parent, QVector<QJsonObject> list)
    : CQJsonModel(parent, list), workflow(nullptr), settingcontroller(nullptr) {

}
bool CAlgorithmSettingsModel::validateAll(){
   int count = mRootItem->getChilds()->size();
   for(int row = 0; row < count; row++)
   {
       QJsonObject data;
       CQJsonTreeItem* parentItem = mRootItem->getChilds()->value(row);
       CQJsonTreeItem* tempItem;
       int size = parentItem->getChilds()->size();
       for(int i = 0; i < size; i++)
       {
         tempItem= parentItem->getChilds()->value(i);
         QJsonObject o = tempItem->toJson();
         QString key = o.keys().at(0);
         data.insert(key, o.take(key));
       }
        if (workflow->getStep(row)->ValidateParameters(&data) == false) {
            CLogController::instance().frameworkMessage("Error: " + workflow->getStep(row)->Name() + " Invalid Parameter");
         return false;
        }else{
            workflow->getStep(row)->getAlgorithm()->setParameters(data);
        }
   }
   return true;
}

void CAlgorithmSettingsModel::saveSettings(int row, QUrl filename) {
    QJsonObject data;
    CQJsonTreeItem* parentItem = mRootItem->getChilds()->value(row);
    CQJsonTreeItem* tempItem;
    int size = parentItem->getChilds()->size();
    for(int i = 0; i < size; i++)
    {
      tempItem= parentItem->getChilds()->value(i);
      QJsonObject o = tempItem->toJson();
      QString key = o.keys().at(0);
      data.insert(key, o.take(key));
    }
    if (filename.isEmpty() == true) {
        filename.setPassword("a" + workflow->getStep(row)->Name());
    }
    if (workflow->getStep(row)->ValidateParameters(&data)) {
        emit saveQJson(data, filename);
        workflow->getStep(row)->getAlgorithm()->setParameters(data);
    }
    else{
        QString pluginM = workflow->getStep(row)->Name();
        pluginM.append(" parameter not Valid!");
        CLogController::instance().frameworkMessage(pluginM);
    }
}
void CAlgorithmSettingsModel::saveSettingsEx(int row, QUrl filename) {
    QJsonObject data;
    CQJsonTreeItem* parentItem = mRootItem->getChilds()->value(row);
    CQJsonTreeItem* tempItem;
    int size = parentItem->getChilds()->size();
    for(int i = 0; i < size; i++)
    {
      tempItem= parentItem->getChilds()->value(i);
      QJsonObject o = tempItem->toJson();
      QString key = o.keys().at(0);
      data.insert(key, o.take(key));
    }
    if (filename.isEmpty() == true) {
        filename.setPassword("a" + workflow->getStep(row)->Name());
    }
    if (workflow->getStep(row)->ValidateParameters(&data)) {
        emit saveQJsonEx(data, filename);
        workflow->getStep(row)->getAlgorithm()->setParameters(data);
    }
    else{
        QString pluginM = workflow->getStep(row)->Name();
        pluginM.append(" parameter not Valid!");
        CLogController::instance().frameworkMessage(pluginM);
    }
}

void CAlgorithmSettingsModel::loadSettings(int row, QUrl filename) {
    if (filename.isEmpty() == true) {
       return;
    }
    emit requestQJson(filename);
    int j = mRootItem->getChilds()->size() - 1;
    CQJsonTreeItem* oItem = mRootItem->child(row);
    CQJsonTreeItem* nItem = mRootItem->child(j);
    int ois = oItem->childCount();
    int nis = nItem->childCount();
    if(ois != nis){
      CLogController::instance().frameworkMessage("Error: Invalid Load Data");
      return;
    }

    this->beginResetModel();
    mRootItem->getChilds()->swap(row, j);
    mRootItem->getChilds()->removeAt(j);
    insertName(row);
    this->endResetModel();
      CLogController::instance().frameworkMessage("File: " + filename.toString() + " loaded");

}


void CAlgorithmSettingsModel::algorithmChanged(int step) {
    QJsonObject object = workflow->getStep(step)->GetParameterJson();
    loadQJson(object);
    mRootItem->getChilds()->swap(step, mRootItem->getChilds()->size() - 1);
    mRootItem->getChilds()->removeLast();
    insertName(step);
    workflow->getStep(step)->getAlgorithm()->setParameters(object);
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

    CQJsonTreeItem* tempItem;
    int size = parentItem->getChilds()->size();
    for(int i = 0; i < size; i++)
    {
      tempItem = parentItem->getChilds()->value(i);
      QJsonObject o = tempItem->toJson();
      QString key = o.keys().at(0);
      params.insert(key, o.take(key));
    }

    if(plugin->ValidateParameters(&params))
    {
      plugin->getAlgorithm()->setParameters(params);
    }else{
        QString pluginM = plugin->Name();
        pluginM.append(" parameter not Valid!");
        CLogController::instance().frameworkMessage(pluginM);
    }
    return true;
  }

  return false;
}
void CAlgorithmSettingsModel::insertName(int row)
{
    CQJsonTreeItem* temp = mRootItem->getChilds()->value(row);
    IPlugin* plugin = workflow->getStep(row);
    temp->setKey(plugin ? plugin->Name() : "No plugin loaded");
}
