#include "CTestAlgorithmSelection.h"

#include <QTest>

#include "gui/CMainWindow.h"
#include "workflow/workflow/fourphase/cfourphaseworkflow.h"

bool CTestAlgorithmSelection::setPluginIfValid(QComboBox *comboBox, QString pluginName)
{
    int index = comboBox->findText(pluginName);
    if(index > -1)
    {
      comboBox->setCurrentIndex(index);
      return true;
    }
    return false;
}

void CTestAlgorithmSelection::test()
{
  CMainWindow mw;
  bool featureMatcherFound = false, poseEstimatorFound = false;
  bool depthEstimatorFound = false, fusionPluginFound = false;
  CFourPhaseWorkflow* c4 = new CFourPhaseWorkflow();

  mw.setWorkflow(c4);

  for(QWidget* w : QApplication::allWidgets())
  {

    CStepComboBox* comboBox = qobject_cast<CStepComboBox*>(w);



    if(comboBox)
    {
        if(setPluginIfValid(comboBox, "DummyFeatureMatch_t33"))
        {
          featureMatcherFound = true;
        }
        if(setPluginIfValid(comboBox, "DummyPose_t33"))
        {
          poseEstimatorFound = true;
        }
        if(setPluginIfValid(comboBox, "DummyDepthMap_t33"))
        {
          depthEstimatorFound = true;
        }
        if(setPluginIfValid(comboBox, "DummyFusion_t33"))
        {
          fusionPluginFound = true;
        }
    }
  }

    QVERIFY2(featureMatcherFound, "Dummy feature match plugin not found.");
    QVERIFY2(poseEstimatorFound, "Dummy pose estimator plugin not found.");
    QVERIFY2(depthEstimatorFound, "Dummy depth estimator plugin not found.");
    QVERIFY2(fusionPluginFound, "Dummy fusion plugin not found.");


    QCOMPARE(c4->getStep(0)->Name(), QString("DummyFeatureMatch_t33"));
    QCOMPARE(c4->getStep(1)->Name(), QString("DummyPose_t33"));
    QCOMPARE(c4->getStep(2)->Name(), QString("DummyDepthMap_t33"));
    QCOMPARE(c4->getStep(3)->Name(), QString("DummyFusion_t33"));

}

