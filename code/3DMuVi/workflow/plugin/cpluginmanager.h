#ifndef CPLUGINMANAGER_H
#define CPLUGINMANAGER_H

#include <memory>

#include <QVector>
#include <QDir>
#include <QPluginLoader>

namespace boost
{
  namespace dll
  {
    class shared_library;
  }
}

/*!
 * \class CPluginManager
 * \brief The CPluginManager class
 * \author Nathanael Schneider, Stefan Wolf
 *
 * Verwaltet alle [Plugins](@ref APlugin), die vom System gefunden wurden.
 */
class CPluginManager
{
private:
  CPluginManager();

  static CPluginManager* mInstance;
  std::vector<boost::dll::shared_library*> mLibs;
  QDir mPluginsDir;

public:
  /*!
   * \brief Gibt die Instanz des Plugin Managers zurück.
   * \return Die Instanz des Plugin Managers.
   */
  static CPluginManager* Instance();
  /*!
   * \brief Initialisiert den Manager und lädt alle Plugins.
   * \return Ein Standard returncode.
   */
  qint32 Initialize();

  /*!
   * \brief Gibt alle Plugins zurück, die dem durch den Templateparameter T definierten Typ
   * entsprechen.
   * \return Eine Liste von Plugins.
   *
   * Der Templateparameter T definiert den Typ des Plugins. T muss eine Variable T::symbol besitzen,
   * die den Namen des Symbols der Factory Methode im Plugin definiert. Des Weiteren muss der Typ
   * der Factory Methode als T::Factory definiert sein.
   */
  template<typename T>
  std::vector<std::shared_ptr<T>> getPlugins() const;
};

#include "cpluginmanager_impl.h"

#endif // CPLUGINMANAGER_H
