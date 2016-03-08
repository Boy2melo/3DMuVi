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
          QString name = comboBox->currentText();
          QCOMPARE((name == QString("featureMatch_t34") ||
                    name == QString("pose_t34") ||
                    name == QString("depthMap_t34") ||
                    name == QString("dummyFusion_t34") ),true);
      }
    }

    //initial state
    QCOMPARE(statusBar->currentMessage() , QString("Choose Images"));

    mw.setLoadImage(QUrl(":/data/io/data/testInputDataSet/"));

    //state after images are loaded
    QCOMPARE(statusBar->currentMessage() , QString("Ready to Start"));

    QTest::mouseClick(startButton, Qt::LeftButton,0 ,QPoint(3,3));

    QTest::qWait(1000);

    //state running
    QCOMPARE(statusBar->currentMessage() , QString("Running..."));
    QCOMPARE(startButton->text(), QString("Stop"));
}
