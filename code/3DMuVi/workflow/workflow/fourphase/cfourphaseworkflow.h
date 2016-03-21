#ifndef _H_CFOURPHASEWORKFLOW
#define _H_CFOURPHASEWORKFLOW

#include <workflow/workflow/aworkflow.h>
#include "workflow/plugin/iplugin.h"
#include "workflow/workflow/ccontextdatastore.h"

class CFourPhaseWorkflow : public AWorkflow
{
public:
  /*!
  \brief Initialisiert den Workflow
  */
  CFourPhaseWorkflow();
  /*!
  \brief Gibt reservierten Speicher frei.
  */
  ~CFourPhaseWorkflow();

  /*!
  \brief Gibt die Anzahl an verfügbaren Ausführungsschritten bzw. Algorithmenslots zurück.
  \return Die Anzahl an verfügbaren Ausführungsschritten bzw. Algorihmenslots.
  */
  quint32 getStepCount() const override;

  /*!
  \brief Gibt den erforderlichen Plugintyp für einen gegebenen Slot zurück.
  \param step Der schritt für den der Algorithmus zurückgegeben werden soll.
  \return Der Typ des Plugins für diesen Slot.
  */
  QString getAlgorithmType(const quint32 step) const override;

  /*!
  \brief Versucht ein Plugin einem Schritt zuzuweisen.
  \param step Der Schritt, dem das Plugin zugewiesen werden soll.
  \param plugin Das Plugin, das dem Schritt zugewiesen werden soll.
  \return True, falls der Typ des Plugins zum Schritt passt und das Plugin gesetzt wurde. False
  andernfalls.
  */
  bool trySetStep(const quint32 step, IPlugin* plugin) override;

  /*!
  \brief Gibt das Plugin an einem gegebenen Schritt zurück.
  \param step Der Schritt, dessen Plugin zurück gegeben werden soll.
  \return Das Plugin, das für den gegebenen Schritt gesetzt wurde.
  */
  IPlugin* getStep(const quint32 step) const override;

  /*!
  \brief Prüfe, ob alle benötigten Daten für die Algorithmen durch den Workflow bereitgestellt
  werden.
  \return True, falls die Daten der gesetzten Algorithmen zusammen passen. False andernfalls.
  */
  bool checkAvailableDataTypes() const override;

protected:
  /*!
  \brief Führt die gesetzten Algorithmen auf dem angegeben Datastore aus.
  \param store Der Datastore, der an die Algorithmen zum Datenaustausch übergeben wird.
  */
  void executeAlgorithm(CContextDataStore* store) override;

private:
  IPlugin** mPlugins;

private slots:
  // ReSharper disable once CppFunctionIsNotImplemented
  void SlotAlgorithmFinished(CContextDataStore*);
};

#endif
