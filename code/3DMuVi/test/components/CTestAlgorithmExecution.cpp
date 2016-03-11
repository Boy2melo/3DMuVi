#include "CTestAlgorithmExecution.h"

#include <gui/CMainWindow.h>
#include "workflow/workflow/cworkflowmanager.h"
#include "workflow/plugin/cpluginmanager.h"

void CTestAlgorithmExecution::test(){
    CPluginManager::Instance();
    CWorkflowManager::Instance();
    CMainWindow mw;  
    QStatusBar* statusBar;
    QPushButton* startButton;
    bool featureMatcherFound = false, poseEstimatorFound = false;
    bool depthEstimatorFound = false, fusionPluginFound = false;


    for(QWidget* w : QApplication::allWidgets())
    {
      QStatusBar* tryStatusBar = qobject_cast<QStatusBar*>(w);
      QPushButton* tryStartButton = qobject_cast<QPushButton*>(w);
      CStepComboBox* comboBox = qobject_cast<CStepComboBox*>(w);

      if(tryStatusBar)
      {
          statusBar = tryStatusBar;
      }

      if(tryStartButton && tryStartButton->text() == QString("Start"))
      {
          startButton = tryStartButton;
      }

      if(comboBox)
      {
          if(setPluginIfValid(comboBox, "featureMatch_t34"))
          {
            featureMatcherFound = true;
          }
          if(setPluginIfValid(comboBox, "pose_t34"))
          {
            poseEstimatorFound = true;
          }
          if(setPluginIfValid(comboBox, "depthMap_t34"))
          {
            depthEstimatorFound = true;
          }
          if(setPluginIfValid(comboBox, "dummyFusion_t34"))
          {
            fusionPluginFound = true;
          }
      }
    }

    QVERIFY2(featureMatcherFound, "Dummy feature match plugin not found.");
    QVERIFY2(poseEstimatorFound, "Dummy pose estimator plugin not found.");
    QVERIFY2(depthEstimatorFound, "Dummy depth estimator plugin not found.");
    QVERIFY2(fusionPluginFound, "Dummy fusion plugin not found.");

    //initial state
    QCOMPARE(statusBar->currentMessage() , QString("Choose Images"));

    mw.setLoadImage(QUrl(":/data/io/data/testInputDataSet/"));

    //state after images are loaded
    QCOMPARE(statusBar->currentMessage() , QString("Ready to Start"));

    QTest::mouseClick(startButton, Qt::LeftButton,0 ,QPoint(3,3));

    //state running
    QCOMPARE(statusBar->currentMessage() , QString("Running..."));
    QCOMPARE(startButton->text(), QString("Stop"));

    QTest::qWait(9000);

    //state finished
    QCOMPARE(statusBar->currentMessage() , QString("Workflow finished."));
    QCOMPARE(startButton->text(), QString("Start"));
}

bool CTestAlgorithmExecution::setPluginIfValid(QComboBox* comboBox, QString pluginName)
{
  int index = comboBox->findText(pluginName);
  if(index > -1)
  {
    comboBox->setCurrentIndex(index);
    return true;
  }
  return false;
}
