#include "CTestAlgorithmExecution.h"

#include <gui/CMainWindow.h>
#include "workflow/workflow/cworkflowmanager.h"
#include "workflow/plugin/cpluginmanager.h"

void CTestAlgorithmExecution::test(){
    CPluginManager::Instance();
    CWorkflowManager::Instance();
    CMainWindow mw;


    /*for(QWidget* w : QApplication::allWidgets())
    {
      QGroupBox* groupBox = qobject_cast<QGroupBox*>(w);

      if(groupBox)
      {
        algorithmBoxes.push_back(groupBox);
      }
    }*/
}
