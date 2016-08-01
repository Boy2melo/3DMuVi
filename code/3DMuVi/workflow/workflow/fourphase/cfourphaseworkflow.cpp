#include "cfourphaseworkflow.h"
#include "workflow/plugin/ifeaturematcher.h"
#include "workflow/plugin/iposeestimator.h"
#include "workflow/plugin/idepthestimator.h"
#include "workflow/plugin/ifusor.h"
#include "workflow/plugin/cpluginmanager.h"
#include "logger/controll/CLogController.h"

using namespace std;

CFourPhaseWorkflow::CFourPhaseWorkflow() : AWorkflow()
{
  mfeatureMatchers = CPluginManager::Instance()->getPlugins<IFeatureMatcher>();
  mPoseEstimators = CPluginManager::Instance()->getPlugins<IPoseEstimator>();
  mDepthEstimators = CPluginManager::Instance()->getPlugins<IDepthEstimator>();
  mFusors = CPluginManager::Instance()->getPlugins<IFusor>();
}

CFourPhaseWorkflow::~CFourPhaseWorkflow()
{
}


quint32 CFourPhaseWorkflow::getStepCount() const
{
  return 4;
}

QString CFourPhaseWorkflow::getAlgorithmType(const quint32 step) const
{
  switch(step)
  {
    case 0:
      return IFeatureMatcher::name;
    case 1:
      return IPoseEstimator::name;
    case 2:
      return IDepthEstimator::name;
    case 3:
      return IFusor::name;
    default:
      return "";
  }
}

bool CFourPhaseWorkflow::trySetStep(const quint32 step, const QString& plugin)
{
  if(step >= getStepCount())
  {
    return false;
  }
  else
  {
    switch (step)
    {
      case 0:
      {
        auto featureMatcher = getPluginByName(mfeatureMatchers, plugin);
        if(featureMatcher)
        {
          mCurrentFeatureMatcher = featureMatcher;
          return true;
        }
      }
      break;
      case 1:
      {
        auto poseEstimator = getPluginByName(mPoseEstimators, plugin);
        if(poseEstimator)
        {
          mCurrentPoseEstimator = poseEstimator;
          return true;
        }
      }
      break;
      case 2:
      {
        auto depthEstimator = getPluginByName(mDepthEstimators, plugin);
        if(depthEstimator)
        {
          mCurrentDepthEstimator = depthEstimator;
          return true;
        }
      }
      break;
      case 3:
      {
        auto fusor = getPluginByName(mFusors, plugin);
        if(fusor)
        {
          mCurrentFusor = fusor;
          return true;
        }
      }
      break;
    }

    return false;
  }
}

QStringList CFourPhaseWorkflow::getAvailablePlugins(const quint32 step)
{
  switch(step)
  {
    case 0:
      return getPluginNames(mfeatureMatchers);
    case 1:
      return getPluginNames(mPoseEstimators);
    case 2:
      return getPluginNames(mDepthEstimators);
    case 3:
      return getPluginNames(mFusors);
    default:
      return QStringList();
  }
}

std::shared_ptr<IAlgorithm> CFourPhaseWorkflow::getStep(const quint32 step) const
{
  switch(step)
  {
    case 0:
    {
      return mCurrentFeatureMatcher;
    }
    break;

    case 1:
    {
      return mCurrentPoseEstimator;
    }
    break;

    case 2:
    {
      return mCurrentDepthEstimator;
    }
    break;

    case 3:
    {
      return mCurrentFusor;
    }
    break;
  }

  return std::shared_ptr<IAlgorithm>();
}

void CFourPhaseWorkflow::executeAlgorithm(CContextDataStore* store)
{
  if(store->getData<CInputDataSet>() == nullptr)
  {
    emit sigDataStoreFinished(store);
    return;
  }

  mCurrentFeatureMatcher->setImages(store->getData<CInputDataSet>());
  if(!executeSingleAlgorithm(mCurrentFeatureMatcher, store))
  {
    return;
  }
  store->appendData(mCurrentFeatureMatcher->getFeatureMatches());

  mCurrentPoseEstimator->setImages(store->getData<CInputDataSet>());
  mCurrentPoseEstimator->setFeatureMatches(store->getData<CDataFeature>());
  if(!executeSingleAlgorithm(mCurrentPoseEstimator, store))
  {
    return;
  }
  store->appendData(mCurrentPoseEstimator->getPoses());

  mCurrentDepthEstimator->setImages(store->getData<CInputDataSet>());
  mCurrentDepthEstimator->setPoses(store->getData<CDataPose>());
  if(!executeSingleAlgorithm(mCurrentDepthEstimator, store))
  {
    return;
  }
  store->appendData(mCurrentDepthEstimator->getDepthMaps());

  mCurrentFusor->setImages(store->getData<CInputDataSet>());
  mCurrentFusor->setDepthMaps(store->getData<CDataDepth>());
  if(!executeSingleAlgorithm(mCurrentFusor, store))
  {
    return;
  }
  store->appendData(mCurrentFusor->getFusion());

  emit sigDataStoreFinished(store);
}

template<typename T>
shared_ptr<T> CFourPhaseWorkflow::getPluginByName(const vector<shared_ptr<T>>& pluginList,
                                                  const QString& name)
{
  for(auto p : pluginList)
  {
    if(p->getName() == name)
    {
      return p;
    }
  }
  return nullptr;
}

template<typename T>
QStringList CFourPhaseWorkflow::getPluginNames(const std::vector<std::shared_ptr<T>>& pluginList)
{
  QStringList names;

  for(auto p : pluginList)
  {
    names += p->getName();
  }

  return names;
}
