#include "CTestAlgorithmSelection.h"

#include <QTest>

#include "gui/CMainWindow.h"
#include "workflow/workflow/fourphase/cfourphaseworkflow.h"

void CTestAlgorithmSelection::test()
{
  CMainWindow mw;
  bool featureMatcherFound = false, poseEstimatorFound = false;
  bool depthEstimatorFound = false, fusionPluginFound = false;
  QComboBox* featureMatcherCB = nullptr, *poseEstimatorCB = nullptr;
  QComboBox* depthEstimatorCB = nullptr, *fusionPluginCB = nullptr;
  int featureMatcherIndex = -1, poseEstimatorIndex = -1;
  int depthEstimatorIndex = -1, fusionPluginIndex = -1;
  CFourPhaseWorkflow* c4 = new CFourPhaseWorkflow();

  mw.setWorkflow(c4);

  for(QWidget* w : QApplication::allWidgets())
  {

    CStepComboBox* comboBox = qobject_cast<CStepComboBox*>(w);

    if(comboBox)
    {
      int index = comboBox->findText("DummyFeatureMatch_t33");
      if(index > -1)
      {
        featureMatcherFound = true;
        featureMatcherCB = comboBox;
        featureMatcherIndex = index;
      }

      index = comboBox->findText("DummyPose_t33");
      if(index > -1)
      {
        poseEstimatorFound = true;
        poseEstimatorCB = comboBox;
        poseEstimatorIndex = index;
      }

      index = comboBox->findText("DummyDepthMap_t33");
      if(index > -1)
      {
        depthEstimatorFound = true;
        depthEstimatorCB = comboBox;
        depthEstimatorIndex = index;
      }

      index = comboBox->findText("DummyFusion_t33");
      if(index > -1)
      {
        fusionPluginFound = true;
        fusionPluginCB = comboBox;
        fusionPluginIndex = index;
      }
    }
  }

    QVERIFY2(featureMatcherFound, "Dummy feature match plugin not found.");
    QVERIFY2(poseEstimatorFound, "Dummy pose estimator plugin not found.");
    QVERIFY2(depthEstimatorFound, "Dummy depth estimator plugin not found.");
    QVERIFY2(fusionPluginFound, "Dummy fusion plugin not found.");

    featureMatcherCB->setCurrentIndex(featureMatcherIndex);
    poseEstimatorCB->setCurrentIndex(poseEstimatorIndex);
    depthEstimatorCB->setCurrentIndex(depthEstimatorIndex);
    fusionPluginCB->setCurrentIndex(fusionPluginIndex);

    QCOMPARE(c4->getStep(0)->Name(), QString("DummyFeatureMatch_t33"));
    QCOMPARE(c4->getStep(1)->Name(), QString("DummyPose_t33"));
    QCOMPARE(c4->getStep(2)->Name(), QString("DummyDepthMap_t33"));
    QCOMPARE(c4->getStep(3)->Name(), QString("DummyFusion_t33"));

}

