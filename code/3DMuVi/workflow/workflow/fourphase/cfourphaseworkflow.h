#ifndef _H_CFOURPHASEWORKFLOW
#define _H_CFOURPHASEWORKFLOW

#include <workflow/workflow/aworkflow.h>
#include "workflow/workflow/ccontextdatastore.h"

class IFeatureMatcher;
class IPoseEstimator;
class IDepthEstimator;
class IFusor;

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
  virtual bool trySetStep(const quint32 step, const QString& plugin) override;

  /*!
  \brief Gibt eine Liste aller verfügbaren Plugins für einen Schritt zurück.
  \param step Der Schritt, für den die Liste erstellt werden soll.
  \return Eine Liste aller verfügbaren Plugins, die mit dem gewählten Schritt kompatibel sind.
  */
  virtual QStringList getAvailablePlugins(const quint32 step) override;

  /*!
  \brief Gibt das Plugin an einem gegebenen Schritt zurück.
  \param step Der Schritt, dessen Plugin zurück gegeben werden soll.
  \return Das Plugin, das für den gegebenen Schritt gesetzt wurde.
  */
  std::shared_ptr<IAlgorithm> getStep(const quint32 step) const override;

protected:
  /*!
  \brief Führt die gesetzten Algorithmen auf dem angegeben Datastore aus.
  \param store Der Datastore, der an die Algorithmen zum Datenaustausch übergeben wird.
  */
  void executeAlgorithm(CContextDataStore* store) override;

private:
  template<typename T>
  static std::shared_ptr<T> getPluginByName(const std::vector<std::shared_ptr<T>>& pluginList,
                                            const QString& name);
  template<typename T>
  static QStringList getPluginNames(const std::vector<std::shared_ptr<T> >& pluginList);

  std::vector<std::shared_ptr<IFeatureMatcher>> mfeatureMatchers;
  std::vector<std::shared_ptr<IPoseEstimator>> mPoseEstimators;
  std::vector<std::shared_ptr<IDepthEstimator>> mDepthEstimators;
  std::vector<std::shared_ptr<IFusor>> mFusors;

  std::shared_ptr<IFeatureMatcher> mCurrentFeatureMatcher;
  std::shared_ptr<IPoseEstimator> mCurrentPoseEstimator;
  std::shared_ptr<IDepthEstimator> mCurrentDepthEstimator;
  std::shared_ptr<IFusor> mCurrentFusor;

private slots:
  // ReSharper disable once CppFunctionIsNotImplemented
  void SlotAlgorithmFinished(CContextDataStore*);
};

#endif
