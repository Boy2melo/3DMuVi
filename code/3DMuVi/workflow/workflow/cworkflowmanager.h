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
  @brief Gibt die Instanz des Workflowmanagers zurück.
  @return Die Instanz des Workflowmanagers.
  */
  static CWorkflowManager* Instance();

  /**
  @brief Gibt eine Liste aller verfügbaren Workflows zurück.
  @return Eine Liste der Namen aller verfügbaren Workflows.
  @remark Ist im Moment noch hardcoded.
  */
  QVector<QString> getAvailableWorkflows() const;

  /**
  @brief Gibt eine Instanz eines spezifischen Workflows zurück.
  @param workflow Der Name des geforderten Workflows.
  @return Eine Instanz eines spezifischen Workflow.
  @remark Momentan wird bei jedem Aufruf ein neues Objekt auf dem Heap erzeugt mangels zentraler
  Verwaltung der Workflows.
  */
  AWorkflow* getWorkflow(QString workflow) const;
};

#endif // CWORKFLOWMANAGER_H
