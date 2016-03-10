#include <QTest>

class CTestCContextDataStore : public QObject {
    Q_OBJECT;

    private slots:
    /**
    \brief Teste die InitializeFromStorage funktion
    */
    void testInitializeFromStorage();

    /**
    \brief Teste den Datenzugriff

    Testet getData, createData und appendData
    */
    void testDataAccess();

    /**
    \brief Teste das Abbruchflag
    */
    void testAbortFlag();

    /**
    \brief Teste die berechnungsschritte
    */
    void testCalculateStep();

    /**
    \brief Teste das Anwenden auf ein DataView
    */
    void testApplyToDataView();
};

