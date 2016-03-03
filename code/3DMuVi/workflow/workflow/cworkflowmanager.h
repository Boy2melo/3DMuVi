#ifndef CWORKFLOWMANAGER_H
#define CWORKFLOWMANAGER_H

#include <QVector>
#include "workflow/workflow/aworkflow.h"

class CWorkflowManager {
private:
    CWorkflowManager();

public:
    /**
    @brief Gibt die Instanz des Workflowmanagers zurück
    */
    static CWorkflowManager* Instance();

    /**
    @brief Gibt eine Liste aller verfügbaren Workflows zurück
    @remark ist im moment noch hardcoded
    */
    QVector<QString> getAvailableWorkflows() const;

    /**
    @brief Gibt eine Instanz eines spezifischen algorithmus zurück
    @remark Momentan wird bei jedem aufruf ein neues Objekt auf dem Heap erzeugt mangels zentraler verwaltung der Workflows
    */
    AWorkflow* getWorkflow(QString workflow) const;
};

#endif // CWORKFLOWMANAGER_H
