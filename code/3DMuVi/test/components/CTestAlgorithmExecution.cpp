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
    QComboBox* featureMatcherCB = nullptr, *poseEstimatorCB = nullptr;
    QComboBox* depthEstimatorCB = nullptr, *fusionPluginCB = nullptr;
    int featureMatcherIndex = -1, poseEstimatorIndex = -1;
    int depthEstimatorIndex = -1, fusionPluginIndex = -1;

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
          int index = comboBox->findText("featureMatch_t34");
          if(index > -1)
          {
            featureMatcherFound = true;
            featureMatcherCB = comboBox;
            featureMatcherIndex = index;
          }

          index = comboBox->findText("pose_t34");
          if(index > -1)
          {
            poseEstimatorFound = true;
            poseEstimatorCB = comboBox;
            poseEstimatorIndex = index;
          }

          index = comboBox->findText("depthMap_t34");
          if(index > -1)
          {
            depthEstimatorFound = true;
            depthEstimatorCB = comboBox;
            depthEstimatorIndex = index;
          }

          index = comboBox->findText("dummyFusion_t34");
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
