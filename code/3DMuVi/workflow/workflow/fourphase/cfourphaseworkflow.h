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
  \brief Gibt die Anzahl an verf�gbaren Ausf�hrungsschritten bzw. Algorithmenslots zur�ck.
  \return Die Anzahl an verf�gbaren Ausf�hrungsschritten bzw. Algorihmenslots.
  */
  quint32 getStepCount() const override;


  /*!
  \brief Gibt den erforderlichen Plugintyp f�r einen gegebenen Slot zur�ck.
  \param step Der schritt f�r den der Algorithmus zur�ckgegeben werden soll.
  \return Der Typ des Plugins f�r diesen Slot.
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
  \brief Gibt eine Liste aller verf�gbaren Plugins f�r einen Schritt zur�ck.
  \param step Der Schritt, f�r den die Liste erstellt werden soll.
  \return Eine Liste aller verf�gbaren Plugins, die mit dem gew�hlten Schritt kompatibel sind.
  */
  virtual QStringList getAvailablePlugins(const quint32 step) override;

  /*!
  \brief Gibt das Plugin an einem gegebenen Schritt zur�ck.
  \param step Der Schritt, dessen Plugin zur�ck gegeben werden soll.
  \return Das Plugin, das f�r den gegebenen Schritt gesetzt wurde.
  */
  std::shared_ptr<IAlgorithm> getStep(const quint32 step) const override;

protected:
  /*!
  \brief F�hrt die gesetzten Algorithmen auf dem angegeben Datastore aus.
  \param store Der Datastore, der an die Algorithmen zum Datenaustausch �bergeben wird.
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
