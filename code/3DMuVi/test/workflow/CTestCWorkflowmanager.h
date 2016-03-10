#include <QTest>

class CTestCWorkflowmanager : public QObject{
    Q_OBJECT;

    private slots:
    /**
    \brief Teste ob die Instanz invariant bleibt
    */
    void testInstance();

    /**
    \brief Teste, ob instanzen erfolgreich generiert und zurückgegeben werden
    */
    void testInstanciate();
};

