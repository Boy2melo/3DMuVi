#include "cworkflowmanager.h"
#include "fourphase/cfourphaseworkflow.h"

CWorkflowManager::CWorkflowManager() {}

CWorkflowManager* CWorkflowManager::Instance()
{
  static CWorkflowManager instance;
  return &instance;
}

QVector<QString> CWorkflowManager::getAvailableWorkflows() const
{
  // Hardcoded list of workflows atm, as they are not modular
  QVector<QString> workflows;
  workflows.push_back("4Phase Workflow");

  return workflows;
}

AWorkflow* CWorkflowManager::getWorkflow(QString workflow) const
{
  // Hardcoded list of workflows atm, as they are not modular

  if(workflow == "4Phase Workflow")
  {
    return new CFourPhaseWorkflow();
  }

  return nullptr;
}
