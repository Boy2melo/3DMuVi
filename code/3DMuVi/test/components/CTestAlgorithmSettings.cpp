#include "CTestAlgorithmSettings.h"

#include <gui/AlgorithmSettings/CAlgorithmSettingsView.h>
#include <workflow/workflow/cworkflowmanager.h>
#include <workflow/plugin/cpluginmanager.h>

QJsonObject* CTestAlgorithmSettings::mpSettings = nullptr;
const int CTestAlgorithmSettings::TEST_VALUE_VALID = 33;
const int CTestAlgorithmSettings::TEST_VALUE_INVALID = 66;

void CTestAlgorithmSettings::setParameters(QJsonObject* settings)
{
  mpSettings = settings;
}

void CTestAlgorithmSettings::test()
{
  AWorkflow* workflow = CWorkflowManager::Instance()->getWorkflow("4Phase Workflow");
  CAlgorithmSettingsView* settingsView = new CAlgorithmSettingsView;
  QAbstractItemModel* model = static_cast<QTreeView*>(settingsView)->model();
  QVector<IPlugin*> plugins = CPluginManager::Instance()->getPlugins();
  QModelIndex index;
  bool pluginFound = false;

  for(IPlugin* p : plugins)
  {
    if(p->Name() == "AlgorithmSettingsTest")
    {
      workflow->trySetStep(0, p);
      pluginFound = true;
      break;
    }
  }
  QCOMPARE(pluginFound, true);

  if(pluginFound)
  {
    settingsView->setWorkflow(*workflow);

    index = model->index(0, 0, settingsView->rootIndex());
    index = model->index(0, 1, index);

    mpSettings = nullptr;
    model->setData(index, QVariant(TEST_VALUE_VALID));
    QVERIFY(mpSettings != nullptr);
    if(mpSettings)
    {
      QCOMPARE(mpSettings->value("parameter").toInt(), TEST_VALUE_VALID);
    }

    mpSettings = nullptr;
    model->setData(index, QVariant(TEST_VALUE_INVALID));
    QCOMPARE(mpSettings, static_cast<QJsonObject*>(nullptr));
  }

  delete settingsView;
  delete workflow;
}
