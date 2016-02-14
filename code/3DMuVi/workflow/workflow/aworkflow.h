#ifndef AWORKFLOW_H
#define AWORKFLOW_H

#include "workflow/plugin/iplugin.h"
#include "workflow/workflow/acontextdatastore.h"
#include <QVector>
#include <QSet>
#include <QMutex>
#include <QThread>

// F�hre ein Plugin auf einem Store aus mit frei w�hlbarem Callback bei abschluss.
#define __RUN_ALGORITHM(PLUGIN, CALLBACK) \
    IAlgorithm* algorithm = PLUGIN->getAlgorithm(); \
    if(!algorithm->IsBusy() && !store->IsAborted()){ \
        store->incCalculationStep(); \
        algorithm->run(store, [this](AContextDataStore *store){CALLBACK}); \
    } else { \
        emit sigDataStoreFinished(store); \
    }

/*!
\class AWorkflow
\brief The AWorkflow class
\author Nathanael Schneider

Enth�lt die Grundlegenden Definitionen f�r einen Workflow
*/
class AWorkflow : public QObject {
    Q_OBJECT; //< ';' only there for correct auto format

private:
    QMutex mMutex;
    QSet<QString> mRunningAlgorithms;

public:
    virtual ~AWorkflow() {}

    /*!
    \brief Die Anzahl an verf�gbaren Ausf�hrungsschritten bzw. Algorithmenslots
    */
    virtual quint32 getStepCount() const = 0;
    /*!
    \brief Gibt den erforderlichen typ an plugin f�r einen gegebenen Slot zur�ck
    \param step Der schritt f�r den der Algorithmus zur�ckgegeben werden soll
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
    \brief Gibt das Plugin an einem gegebenen Schritt zur�ck
    */
    virtual IPlugin* getStep(const quint32 step) const = 0;
    /*!
    \brief Gibt eine Liste aller Datastores zur�ck, die in diesem Workflow angelegt wurden.
    */
    virtual QList<AContextDataStore*> getDataStores() const = 0;
    /*!
    \brief F�gt dem Workflow einen neuen DataStore hinzu und gibt ihn zur�ck
    */
    virtual AContextDataStore* addDataStore() = 0;
    /*!
    \brief Entfernt einen Datastore aus der Verwaltung des Workflows. Das Objekt wird ebenfalls aus dem Heap gel�scht.
    */
    virtual bool removeDataStore(QString id) = 0;

    /*!
    \brief F�hre den Workflow auf einem gegebenen DataStore aus
    */
    bool run(const QString storeId);
    /*!
    \brief Gib den aktuellen ausf�hrungsschritt f�r einen Datastore zur�ck
    */
    virtual qint32 getState(const QString storeId) const = 0;
    /*!
    \brief Beende das Durchreichen an den n�chsten Schritt f�r einen Datastore
    */
    virtual void stop(const QString storeId) = 0;
    /*!
    \brief Pr�fe, ob alle ben�tigten Daten f�r die Algorithmen durch den Workflow bereitgestellt werden
    */
    virtual bool checkAvailableDataTypes() const = 0;

protected:
    virtual AContextDataStore *FindStore(QString id) const = 0;

    protected slots:
    virtual void executeAlgorithm(AContextDataStore* store) = 0;

signals:
    /*!
    \brief Wird aufgerufen, sobald ein Datastore die Ausf�hrung beendet hat oder nicht weitergereicht wurde
    */
    void sigDataStoreFinished(AContextDataStore *store);

    /*!
    \brief For internal use only. Do not connect
    
    Wird aufgerufen, sobald die Plugins gestartet werden k�nnen. Wird ben�tigt, um die Plugins in einem 
    separaten Thread auszuf�hren.
    */
    void sigExecuteAlgorithm(AContextDataStore *store);
};

#endif // AWORKFLOW_H
