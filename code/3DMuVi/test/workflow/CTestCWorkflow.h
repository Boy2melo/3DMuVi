#pragma once
#include <QTest>

class CTestCWorkflow : public QObject {
    Q_OBJECT;

    private slots:
    /**
    \brief Teste das Zuweisen von Algorithmen in die Slots
    */
    void TestAlgorithmAssign();
    /**
    \brief Teste den Zugriff auf die Datastores
    */
    void TestDataStores();
    /**
    \brief Teste die Typvalidierung
    */
    void TestDataTypeValidation();

    /**
    \brief Teste die Ausführung der Algorithmen
    */
    void TestExecuteAlgorithms();
};

