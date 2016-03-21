#ifndef CWORKFLOWMANAGER_H
#define CWORKFLOWMANAGER_H

#include <QVector>
#include "workflow/workflow/aworkflow.h"

class CWorkflowManager
{
private:
  CWorkflowManager();

public:
  /**
  @brief Gibt die Instanz des Workflowmanagers zur�ck.
  @return Die Instanz des Workflowmanagers.
  */
  static CWorkflowManager* Instance();

  /**
  @brief Gibt eine Liste aller verf�gbaren Workflows zur�ck.
  @return Eine Liste der Namen aller verf�gbaren Workflows.
  @remark Ist im Moment noch hardcoded.
  */
  QVector<QString> getAvailableWorkflows() const;

  /**
  @brief Gibt eine Instanz eines spezifischen Workflows zur�ck.
  @param workflow Der Name des geforderten Workflows.
  @return Eine Instanz eines spezifischen Workflow.
  @remark Momentan wird bei jedem Aufruf ein neues Objekt auf dem Heap erzeugt mangels zentraler
  Verwaltung der Workflows.
  */
  AWorkflow* getWorkflow(QString workflow) const;
};

#endif // CWORKFLOWMANAGER_H
