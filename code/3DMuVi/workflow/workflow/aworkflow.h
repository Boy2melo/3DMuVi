#ifndef AWORKFLOW_H
#define AWORKFLOW_H

#include "workflow/plugin/iplugin.h"
#include "ccontextdatastore.h"
#include <QVector>
#include <QSet>
#include <QMutex>
#include <QThread>

// Führe ein Plugin auf einem Store aus mit frei wählbarem Callback bei Abschluss.
#define __RUN_ALGORITHM(PLUGIN, CALLBACK) \
  IAlgorithm* algorithm = PLUGIN->getAlgorithm(); \
  algorithm->setLogger(&CLogController::instance()); \
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

Enthält die Grundlegenden Definitionen für einen Workflow.
*/
class AWorkflow : public QObject
{
  Q_OBJECT; //< ';' only there for correct auto format

public:
  /*!
  \brief Löscht enthaltene Datastores.
  */
  virtual ~AWorkflow();

  /*!
  \brief Gibt die Anzahl an verfügbaren Ausführungsschritten bzw. Algorithmenslots zurück.
  \return Die Anzahl an verfügbaren Ausführungsschritten bzw. Algorithmenslots.
  */
  virtual quint32 getStepCount() const = 0;
  /*!
  \brief Gibt den erforderlichen Plugintyp für einen gegebenen Slot zurück.
  \param step Der Schritt, für den der Plugintyp zurückgegeben werden soll.
  \return Der Plugintyp, der für den angegeben Schritt erforderlich ist.
  */
  virtual QString getAlgorithmType(const quint32 step) const = 0;
  /*!
  \brief Versucht ein Plugin einem Schritt zuzuweisen.
  \param step Der Schritt, dem das Plugin zugewiesen werden soll.
  \param plugin Das Plugin, das dem Schritt zugewiesen werden soll.
  \return True, falls der Typ des Plugins zum Schritt passt und das Plugin gesetzt wurde. False
  andernfalls.
  */
  virtual bool trySetStep(const quint32 step, IPlugin* plugin) = 0;
  /*!
  \brief Gibt das gesetze Plugin für einem gegebenen Schritt zurück.
  \param step Der Schritt, für den das Plugin zurück gegeben werden soll.
  \return Das gesetzte Plugin.
  */
  virtual IPlugin* getStep(const quint32 step) const = 0;
  /*!
  \brief Gibt eine Liste aller Datastores zurück, die in diesem Workflow angelegt wurden.
  \return Eine Liste aller Datastore, die in diesem Workflow angelegt wurden.
  */
  QList<CContextDataStore*> getDataStores() const;
  /*!
  \brief Fügt dem Workflow einen neuen Datastore hinzu und gibt ihn zurück.
  \return Der neu erstellte Datastore.
  */
  CContextDataStore* addDataStore() const;
  /*!
  \brief Entfernt einen Datastore aus der Verwaltung des Workflows. Das Objekt wird ebenfalls aus
  dem Heap gelöscht.
  \param id Die ID des zu entfernenden Datastores.
  \return True, falls das Löschen erfolgreich war. False andernfalls.
  */
  bool removeDataStore(QString id) const;

  /*!
  \brief Führe den Workflow auf einem gegebenen DataStore aus.
  \param storeId Die ID des Datastores, auf dem der Workflow ausgeführt werden soll.
  \param multiThread Der Workflow wird auf einem neuen Thread ausgeführt, falls true.
  \return True, falls die Ausführung erfolgreich gestartet wurde. False andernfalls.
  */
  bool run(const QString storeId, bool multiThread = true);
  /*!
  \brief Gib den aktuellen Ausführungsschritt für einen Datastore zurück.
  \param storeId Die ID des Datastores, für den der Ausführungsschritt zurück gegeben werden soll.
  \return Der aktuelle Ausführungsschritt.
  */
  qint32 getState(const QString storeId) const;
  /*!
  \brief Beende das Durchreichen an den nächsten Schritt für einen Datastore.
  \param storeId Die ID des Datastores, dessen Ausführung angehalten werden soll.
  */
  void stop(const QString storeId) const;
  /*!
  \brief Prüfe, ob alle benötigten Daten für die Algorithmen durch den Workflow bereitgestellt
  werden.
  \return True, falls die benötigten Daten stimmen. False andernfalls.
  */
  virtual bool checkAvailableDataTypes() const = 0;

  /*!
  \brief Initialisiert den Workflow.
  */
  AWorkflow();

signals:
  /*!
  \brief Wird aufgerufen, sobald ein Datastore die Ausführung beendet hat oder nicht weitergereicht
  wurde.
  \param store Der betreffone Datastore.
  */
  void sigDataStoreFinished(CContextDataStore* store);

protected:
  QList<CContextDataStore*>* mDataStores;

  /*!
  \brief Gibt einen Pointer auf den Datastore zurück, der die angegebene ID hat.
  \param id Die ID der gesuchten Datastores.
  \return Der gesucht Datastore oder nullptr, falls er nicht gefunden wurde.
  */
  CContextDataStore* FindStore(QString id) const;

protected slots:
  /*!
  \brief Führt die gesetzten Algorithmen auf dem angegeben Datastore aus.
  \param store Der Datastore, der an die Algorithmen zum Datenaustausch übergeben wird.
  */
  virtual void executeAlgorithm(CContextDataStore* store) = 0;

private:
  QMutex mMutex;
  QSet<QString> mRunningAlgorithms;
};

#endif // AWORKFLOW_H
