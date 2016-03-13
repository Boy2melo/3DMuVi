#include "CTestCWorkflowmanager.h"
#include "workflow/workflow/cworkflowmanager.h"


void CTestCWorkflowmanager::testInstance() {
    auto manager1 = CWorkflowManager::Instance();
    auto manager2 = CWorkflowManager::Instance();

    QCOMPARE(manager1, manager2);
}

void CTestCWorkflowmanager::testInstanciate() {
    auto manager = CWorkflowManager::Instance();
    auto workflows = manager->getAvailableWorkflows();

    for (int i = 0; i < workflows.count(); i++) {
        auto workflow = manager->getWorkflow(workflows.at(i));

        if (workflow == nullptr) {
            QFAIL("A Workflow reported as available was not instanciated");
        }

        delete workflow;
    }
}
