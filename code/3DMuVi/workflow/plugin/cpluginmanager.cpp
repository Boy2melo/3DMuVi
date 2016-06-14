#include "cpluginmanager.h"
#include <QtCore/qcoreapplication.h>

//TODO: store plugins as shared_ptr and make them reloadable

CPluginManager* CPluginManager::mInstance = nullptr;

CPluginManager::CPluginManager() {}

CPluginManager* CPluginManager::Instance()
{
  if(mInstance == nullptr)
  {
    mInstance = new CPluginManager();
    mInstance->Initialize();
  }

  return mInstance;
}

QVector<IPlugin*> CPluginManager::getPlugins() const
{
  return mPlugins;
}

qint32 CPluginManager::Initialize()
{
  mPlugins.clear();

  mPluginsDir = QDir(qApp->applicationDirPath());
#if defined(Q_OS_WIN)
  if(mPluginsDir.dirName() == "debug" || mPluginsDir.dirName() == "release")
  {
    mPluginsDir.cdUp();
  }
#elif defined(Q_OS_MAC)
  if(mPluginsDir.dirName() == "MacOS")
  {
    mPluginsDir.cdUp();
    mPluginsDir.cdUp();
    mPluginsDir.cdUp();
  }
#endif
  mPluginsDir.cd("plugins");

  for(QString fileName : mPluginsDir.entryList(QDir::Files))
  {
    QPluginLoader loader(mPluginsDir.absoluteFilePath(fileName));
    QObject* plugin = loader.instance();
    auto pPlugin = qobject_cast<IPlugin*>(plugin);

    if(pPlugin != nullptr)
    {
      pPlugin->Initialize(&loader);
      mPlugins.push_back(pPlugin);
    }
  }

  auto size = mPlugins.size();
  return size;
}

QVector<IPlugin*> CPluginManager::getPlugins(QString type) const
{
  QVector<IPlugin*> result;

  for(IPlugin* plugin : mPlugins)
  {
    auto pluginType = plugin->GetPluginType();
    if(pluginType == type)
    {
      result.push_back(plugin);
    }
  }

  return result;
}
