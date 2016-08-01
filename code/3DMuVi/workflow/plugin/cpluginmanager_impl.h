#ifndef CPLUGINMANAGER_H
#error "Don't include this file. Include the real header instead."
#endif

#include <boost/dll/shared_library.hpp>

#include <QVector>

template<typename T>
std::vector<std::shared_ptr<T>> CPluginManager::getPlugins() const
{
  std::vector<std::shared_ptr<T>> result;

  for(boost::dll::shared_library* lib : mLibs)
  {
    if(lib->has(T::symbol))
    {
      typename T::Factory* fac = lib->get<typename T::Factory*>(T::symbol);
      result.push_back(fac());
    }
  }

  return result;
}
