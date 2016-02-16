#ifndef EXAMPLEALGORITHM_H
#define EXAMPLEALGORITHM_H
#include "workflow/plugin/ialgorithm.h"
#include <QJsonObject>
#include <QObject>

class ExampleAlgorithm : public IAlgorithm
{
    Q_OBJECT
private:
    bool mIsBusy;
    CLogController *mLogger;
    QJsonObject *mSettings;

public:
    ExampleAlgorithm();
    /*!
     * \brief Initialisiert einen Logger für den Algorithmus
     */
    void setLogger(CLogController *controller) override;
    /*!
     * \brief Setze die Parameter für den nächsten Durchlauf
     */
    void setParameters(QJsonObject *settings) override;
    /*!
     * \brief Führe dem Algorithmus auf den dem Plugin bekannten Daten aus.
     */
    virtual void run(CContextDataStore* dataStore, std::function<void (CContextDataStore*)> callback) override;

    /*!
    \brief Gibt zurück, ob der Algorithmus zur Zeit mit einer Ausführung beschäftigt ist
    */
    virtual bool IsBusy() const override;

    /*!
    * \brief Eine Liste aller Daten, die als Eingabe benötigt werden.
    * \return Eine Liste aller Daten, die als Eingabe benötigt werden.
    */
    virtual QStringList getInputDataTypes() const override;
    /*!
    * \brief Eine Liste aller Daten, die als Ausgabe erzeugt werden.
    * \return Eine Liste aller Daten, die als Ausgabe erzeugt werden.
    */
    virtual QStringList getOutputDataTypes() const override;
};

#endif // EXAMPLEALGORITHM_H
