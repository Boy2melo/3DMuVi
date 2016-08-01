#include <boost/filesystem/path.hpp>
#include <boost/dll/shared_library.hpp>

#include "cpluginmanager.h"
#include <QtCore/qcoreapplication.h>

//TODO: store plugins as shared_ptr and make them reloadable

#include <iostream>

using namespace boost::filesystem;
using namespace boost::dll;

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

qint32 CPluginManager::Initialize()
{
  mLibs.clear();

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
    path libPath(mPluginsDir.absoluteFilePath(fileName).toStdString());
    auto lib = new shared_library(libPath);

    if(lib->is_loaded())
    {
      mLibs.push_back(lib);
    }
  }

  auto size = mLibs.size();
  return size;
}
