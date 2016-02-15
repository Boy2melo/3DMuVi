#ifndef CPLUGINMANAGER_H
#define CPLUGINMANAGER_H

#include <QVector>
#include <QDir>
#include <QPluginLoader>
#include "iplugin.h"

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
    QVector<IPlugin*> mPlugins;
    QDir mPluginsDir;

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
    QVector<IPlugin*> getPlugins() const;
    /*!
     * \brief Gibt einer Liste aller Plugins von einem gegebenen Typ zurück.
     * \param type Der Typ von Plugins, der gesucht wird
     * \return Alle Plugins vom spezifizierten Typ
     */
    QVector<IPlugin*> getPlugins(QString type) const;

    // Plugin Types:


    /*!
    * \brief Feature Matcher Plugin
    */
#define PT_FeatureMatcher "Feature Matcher"
    /*!
    * \brief Tiefenschätzer Plugin
    */
#define PT_DepthMapper "Depth Mapper"
    /*!
    * \brief Posenschätzung Plugin
    */
#define PT_PoseEstimator "Pose Estimator"
    /*!
    * \brief PCL Rekonstruktionsplugin
    */
#define PT_PclReconstructor "PCL Reconstructor"
};

#endif // CPLUGINMANAGER_H
