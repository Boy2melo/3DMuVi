#ifndef AWORKFLOW_H
#define AWORKFLOW_H

#include "workflow/plugin/iplugin.h"
#include "ccontextdatastore.h"
#include <QVector>
#include <QSet>
#include <QMutex>
#include <QThread>

// Führe ein Plugin auf einem Store aus mit frei wählbarem Callback bei abschluss.
#define __RUN_ALGORITHM(PLUGIN, CALLBACK) \
    IAlgorithm* algorithm = PLUGIN->getAlgorithm(); \
    if(!algorithm->IsBusy() && !store->IsAborted()){ \
        store->incCalculationStep(); \
        algorithm->run(store, [this](CContextDataStore *store){CALLBACK}); \
    } else { \
        emit sigDataStoreFinished(store); \
    }

/*!
\class AWorkflow
\brief The AWorkflow class
\author Nathanael Schneider

Enthält die Grundlegenden Definitionen für einen Workflow
*/
class AWorkflow : public QObject {
    Q_OBJECT; //< ';' only there for correct auto format

private:
    QMutex mMutex;
    QSet<QString> mRunningAlgorithms;

protected:
    QList<CContextDataStore *>* mDataStores;

public:
    virtual ~AWorkflow() {}

    /*!
    \brief Die Anzahl an verfügbaren Ausführungsschritten bzw. Algorithmenslots
    */
    virtual quint32 getStepCount() const = 0;
    /*!
    \brief Gibt den erforderlichen typ an plugin für einen gegebenen Slot zurück
    \param step Der schritt für den der Algorithmus zurückgegeben werden soll
    */
    virtual QString getAlgorithmType(const quint32 step) const = 0;
    /*!
    \brief Versucht ein Plugin einem Schritt zuzuweisen
    \param step Der Schritt, dem das Plugin zugewiesen werden soll
    \param plugin Das Plugin, dass dem Schritt zugewiesen werden soll
    \return True, falls der Typ des Plugins zum Schritt passt und das Plugin gesetzt wurde
    */
    virtual bool trySetStep(const quint32 step, IPlugin* plugin) = 0;
    /*!
    \brief Gibt das Plugin an einem gegebenen Schritt zurück
    */
    virtual IPlugin* getStep(const quint32 step) const = 0;
    /*!
    \brief Gibt eine Liste aller Datastores zurück, die in diesem Workflow angelegt wurden.
    */
    QList<CContextDataStore*> getDataStores() const;
    /*!
    \brief Fügt dem Workflow einen neuen DataStore hinzu und gibt ihn zurück
    */
    CContextDataStore* addDataStore() const;
    /*!
    \brief Entfernt einen Datastore aus der Verwaltung des Workflows. Das Objekt wird ebenfalls aus dem Heap gelöscht.
    */
    bool removeDataStore(QString id) const;

    /*!
    \brief Führe den Workflow auf einem gegebenen DataStore aus
    */
    bool run(const QString storeId);
    /*!
    \brief Gib den aktuellen ausführungsschritt für einen Datastore zurück
    */
    qint32 getState(const QString storeId) const;
    /*!
    \brief Beende das Durchreichen an den nächsten Schritt für einen Datastore
    */
    void stop(const QString storeId) const;
    /*!
    \brief Prüfe, ob alle benötigten Daten für die Algorithmen durch den Workflow bereitgestellt werden
    */
    virtual bool checkAvailableDataTypes() const = 0;

protected:
    CContextDataStore *FindStore(QString id) const;

    AWorkflow();

    protected slots:
    virtual void executeAlgorithm(CContextDataStore* store) = 0;

signals:
    /*!
    \brief Wird aufgerufen, sobald ein Datastore die Ausführung beendet hat oder nicht weitergereicht wurde
    */
    void sigDataStoreFinished(CContextDataStore *store);
};

#endif // AWORKFLOW_H
