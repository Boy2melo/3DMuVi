#ifndef CWORKFLOWMANAGER_H
#define CWORKFLOWMANAGER_H

#include <QVector>
#include "workflow/workflow/aworkflow.h"

class CWorkflowManager
{
private:
    CWorkflowManager();

public:
    static CWorkflowManager* Instance();
    QVector<QString> getAvailableWorkflows() const;
    AWorkflow* getWorkflow(QString workflow) const;
};

#endif // CWORKFLOWMANAGER_H
