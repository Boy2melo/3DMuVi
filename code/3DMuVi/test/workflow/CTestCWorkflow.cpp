#include "CTestCWorkflow.h"
#include "workflow/workflow/fourphase/cfourphaseworkflow.h"
#include "testplugin/plugin.h"
#include <QList>

void CTestCWorkflow::TestAlgorithmAssign() {
    CFourPhaseWorkflow workflow;

    if (workflow.getStepCount() <= 0) {
        QFAIL("Not enough steps in the workflow");
    }

    for (quint32 i = 0; i < workflow.getStepCount(); i++) {
        TestPlugin correct{ workflow.getAlgorithmType(i), "CorrectPlugin",QStringList(), QStringList() };
        TestPlugin wrong{ workflow.getAlgorithmType(i).append("falseOne"), "FalsePlugin", QStringList(), QStringList() };

        QCOMPARE(workflow.getStep(i), static_cast<IPlugin*>(nullptr));

        auto result = workflow.trySetStep(i, &correct);

        QCOMPARE(result, true);
        QCOMPARE(workflow.getStep(i), &correct);


        result = workflow.trySetStep(i, &wrong);

        QCOMPARE(result, false);
        QCOMPARE(workflow.getStep(i), &correct);

        result = workflow.trySetStep(i, nullptr);

        QCOMPARE(result, true);
        QCOMPARE(workflow.getStep(i), static_cast<IPlugin*>(nullptr));
    }

    auto result = workflow.trySetStep(workflow.getStepCount(), nullptr);

    QCOMPARE(result, false);
}

void CTestCWorkflow::TestDataStores() {
    CFourPhaseWorkflow workflow;

    QCOMPARE(workflow.getDataStores().length(), 0);

    auto store = workflow.addDataStore();

    QCOMPARE(workflow.getDataStores().length(), 1);
    QCOMPARE(workflow.getDataStores().at(0), store);

    workflow.removeDataStore(store->getId());

    QCOMPARE(workflow.getDataStores().length(), 0);
}

void CTestCWorkflow::TestDataTypeValidation() {
    CFourPhaseWorkflow workflow;

    if (workflow.getStepCount() < 2) {
        QFAIL("Not enough steps to test data validation");
    }

    // Use first two steps to test
    QList<TestPlugin*> plugins;

    for (quint32 i = 2; i < workflow.getStepCount(); i++) {
        auto plugin = new TestPlugin{ workflow.getAlgorithmType(i), "Test", QStringList(), QStringList() };
        plugins.push_back(plugin);
        workflow.trySetStep(i, plugin);
    }

    // Test fitting algorithms
    auto plugin1 = new TestPlugin{ workflow.getAlgorithmType(0), "Test", QStringList(),QStringList("Test") };
    plugins.push_back(plugin1);
    auto plugin2 = new TestPlugin{ workflow.getAlgorithmType(1), "Test",QStringList("Test"),QStringList() };
    plugins.push_back(plugin2);

    workflow.trySetStep(0, plugin1);
    workflow.trySetStep(1, plugin2);

    QCOMPARE(workflow.checkAvailableDataTypes(), true);

    plugin2 = new TestPlugin{ workflow.getAlgorithmType(1), "Test",QStringList("Test2"), QStringList() };
    plugins.push_back(plugin2);

    workflow.trySetStep(1, plugin2);

    QCOMPARE(workflow.checkAvailableDataTypes(), false);

    qDeleteAll(plugins);
}

void CTestCWorkflow::TestExecuteAlgorithms() {
    CFourPhaseWorkflow workflow;

    // Use first two steps to test
    QList<TestPlugin*> plugins;

    for (quint32 i = 2; i < workflow.getStepCount(); i++) {
        auto plugin = new TestPlugin{ workflow.getAlgorithmType(i), "Test", QStringList(), QStringList() };
        plugins.push_back(plugin);
        workflow.trySetStep(i, plugin);
    }

    // Test fitting algorithms
    auto plugin1 = new TestPlugin{ workflow.getAlgorithmType(0), "Test", QStringList(), QStringList() };
    plugins.push_back(plugin1);
    auto plugin2 = new TestPlugin{ workflow.getAlgorithmType(1), "Test", QStringList(), QStringList() };
    plugins.push_back(plugin2);

    workflow.trySetStep(0, plugin1);
    workflow.trySetStep(1, plugin2);

    int successCounter = 0;
    plugin1->getAlgorithmCast()->setExecCallback([&successCounter](CContextDataStore*) {
        successCounter++;
    });
    plugin2->getAlgorithmCast()->setExecCallback([&successCounter](CContextDataStore*) {
        successCounter++;
    });

    auto result = workflow.run("", false);

    QCOMPARE(result, false);

    auto store = workflow.addDataStore();

    result = workflow.run(store->getId(), false);

    QCOMPARE(result, true);
    QCOMPARE(successCounter, 2);

    plugin1->getAlgorithmCast()->setExecCallback([&successCounter, &workflow](CContextDataStore* store){
        successCounter++;
        
        workflow.stop(store->getId());

        QCOMPARE(store->IsAborted(), true);
    });

    successCounter = 0;
    workflow.run(store->getId(), false);

    QCOMPARE(successCounter, 1);
    qDeleteAll(plugins);
}
