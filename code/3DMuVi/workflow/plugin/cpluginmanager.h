#ifndef CPLUGINMANAGER_H
#define CPLUGINMANAGER_H

#include <QVector>
#include "workflow/plugin/aplugin.h"

/*!
   \class CPluginManager
 * \brief The CPluginManager class
 * \author Nathanael Schneider
 *
 * Verwaltet alle [Plugins](@ref APlugin), die vom System gefunden wurden.
 */
class CPluginManager
{
private:
    CPluginManager();

    static CPluginManager* mInstance;
    QVector<APlugin*> mPlugins;

public:
    /*!
     * \brief Gibt die Instanz des Plugin Managers zurück
     * \return Gibt die Instanz des Plugin Managers zurück
     */
    static CPluginManager* Instance();
    /*!
     * \brief Initialisiert den Manager und lädt alle Plugins
     * \return Ein Standard returncode
     */
    qint32 Initialize();
    /*!
     * \brief Gibt eine Liste aller Plugins zurück
     * \return Eine Liste aller Plugins
     */
    QVector<APlugin*> getPlugins();
    /*!
     * \brief Gibt einer Liste aller Plugins von einem gegebenen Typ zurück.
     * \param type Der Typ von Plugins, der gesucht wird
     * \return Alle Plugins vom spezifizierten Typ
     */
    QVector<APlugin*> getPlugins(QString type);
};

#endif // CPLUGINMANAGER_H
