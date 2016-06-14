#ifndef AWORKFLOW_H
#define AWORKFLOW_H

#include "workflow/plugin/iplugin.h"
#include "ccontextdatastore.h"
#include <QVector>
#include <QSet>
#include <QMutex>
#include <QThread>

// F�hre ein Plugin auf einem Store aus mit frei w�hlbarem Callback bei Abschluss.
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

Enth�lt die Grundlegenden Definitionen f�r einen Workflow.
*/
class AWorkflow : public QObject
{
  Q_OBJECT; //< ';' only there for correct auto format

public:
  /*!
  \brief L�scht enthaltene Datastores.
  */
  virtual ~AWorkflow();

  /*!
  \brief Gibt die Anzahl an verf�gbaren Ausf�hrungsschritten bzw. Algorithmenslots zur�ck.
  \return Die Anzahl an verf�gbaren Ausf�hrungsschritten bzw. Algorithmenslots.
  */
  virtual quint32 getStepCount() const = 0;
  /*!
  \brief Gibt den erforderlichen Plugintyp f�r einen gegebenen Slot zur�ck.
  \param step Der Schritt, f�r den der Plugintyp zur�ckgegeben werden soll.
  \return Der Plugintyp, der f�r den angegeben Schritt erforderlich ist.
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
  \brief Gibt das gesetze Plugin f�r einem gegebenen Schritt zur�ck.
  \param step Der Schritt, f�r den das Plugin zur�ck gegeben werden soll.
  \return Das gesetzte Plugin.
  */
  virtual IPlugin* getStep(const quint32 step) const = 0;
  /*!
  \brief Gibt eine Liste aller Datastores zur�ck, die in diesem Workflow angelegt wurden.
  \return Eine Liste aller Datastore, die in diesem Workflow angelegt wurden.
  */
  QList<CContextDataStore*> getDataStores() const;
  /*!
  \brief F�gt dem Workflow einen neuen Datastore hinzu und gibt ihn zur�ck.
  \return Der neu erstellte Datastore.
  */
  CContextDataStore* addDataStore() const;
  /*!
  \brief Entfernt einen Datastore aus der Verwaltung des Workflows. Das Objekt wird ebenfalls aus
  dem Heap gel�scht.
  \param id Die ID des zu entfernenden Datastores.
  \return True, falls das L�schen erfolgreich war. False andernfalls.
  */
  bool removeDataStore(QString id) const;

  /*!
  \brief F�hre den Workflow auf einem gegebenen DataStore aus.
  \param storeId Die ID des Datastores, auf dem der Workflow ausgef�hrt werden soll.
  \param multiThread Der Workflow wird auf einem neuen Thread ausgef�hrt, falls true.
  \return True, falls die Ausf�hrung erfolgreich gestartet wurde. False andernfalls.
  */
  bool run(const QString storeId, bool multiThread = true);
  /*!
  \brief Gib den aktuellen Ausf�hrungsschritt f�r einen Datastore zur�ck.
  \param storeId Die ID des Datastores, f�r den der Ausf�hrungsschritt zur�ck gegeben werden soll.
  \return Der aktuelle Ausf�hrungsschritt.
  */
  qint32 getState(const QString storeId) const;
  /*!
  \brief Beende das Durchreichen an den n�chsten Schritt f�r einen Datastore.
  \param storeId Die ID des Datastores, dessen Ausf�hrung angehalten werden soll.
  */
  void stop(const QString storeId) const;
  /*!
  \brief Pr�fe, ob alle ben�tigten Daten f�r die Algorithmen durch den Workflow bereitgestellt
  werden.
  \return True, falls die ben�tigten Daten stimmen. False andernfalls.
  */
  virtual bool checkAvailableDataTypes() const = 0;

  /*!
  \brief Initialisiert den Workflow.
  */
  AWorkflow();

signals:
  /*!
  \brief Wird aufgerufen, sobald ein Datastore die Ausf�hrung beendet hat oder nicht weitergereicht
  wurde.
  \param store Der betreffone Datastore.
  */
  void sigDataStoreFinished(CContextDataStore* store);

protected:
  QList<CContextDataStore*>* mDataStores;

  /*!
  \brief Gibt einen Pointer auf den Datastore zur�ck, der die angegebene ID hat.
  \param id Die ID der gesuchten Datastores.
  \return Der gesucht Datastore oder nullptr, falls er nicht gefunden wurde.
  */
  CContextDataStore* FindStore(QString id) const;

protected slots:
  /*!
  \brief F�hrt die gesetzten Algorithmen auf dem angegeben Datastore aus.
  \param store Der Datastore, der an die Algorithmen zum Datenaustausch �bergeben wird.
  */
  virtual void executeAlgorithm(CContextDataStore* store) = 0;

private:
  QMutex mMutex;
  QSet<QString> mRunningAlgorithms;
};

#endif // AWORKFLOW_H
