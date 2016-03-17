#ifndef ALGORITHM_H
#define ALGORITHM_H
#include <fstream>
#include "workflow/plugin/ialgorithm.h"
#include <QJsonObject>
#include <QObject>
#include "plugin_config.h"


class CLASS_GEN(Algorithm) : public IAlgorithm
{
private:
    bool mIsBusy;
    CLogController *mLogger;
    QJsonObject mQJson;
    QJsonObject *mSettings;
    QStringList mInputTypes;
    QStringList mOutputTypes;
    std::fstream mPoseFileStream;

public:
    CLASS_GEN(Algorithm)();
    /*!
     * \brief Initialisiert einen Logger für den Algorithmus
     */
    void setLogger(CLogController *controller) override;
    /*!
     * \brief Setze die Parameter für den nächsten Durchlauf
     */
    void setParameters(QJsonObject settings) override;
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

    /*!
    \brief Prüfe alle Parameter auf gültige Werte
    \return True falls alle Werte sich in gültigen Grenzen befinden, False andernfalls
    */
    bool ValidateParameters(const QJsonObject*) const;
protected:
    /*!
     * \brief Führe die Konkrete implementierung des Algorithmus aus
     * \param store Der Datastore
     */
    void executeAlgorithm(CContextDataStore* store);

    /*!
     * \brief Wird im Konstruktor aufgerufen. Fülle mInputTypes und mOutputTypes
     */
    virtual void OnInitialize();

private:
    void openFileStream();
};

#endif // ALGORITHM_H
