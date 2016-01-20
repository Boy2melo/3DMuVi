#ifndef APLUGIN_H
#define APLUGIN_H

#include "idataaccess.h"
#include "ialgorithm.h"
#include <QDate>

/*!
 * \class APlugin
 * \brief The APlugin class
 * \author Nathanael Schneider
 *
 * Beschreibt ein einzelnes Plugin, welches einen einzelnen [Algorithmus](@ref IAlgorithm) kapselt.
 */
class APlugin
{    
public:
    /*!
     * \brief DataAccess
     * \return Instanz auf eine Implementierung von [IDataAccess](@ref IDataAccess)
     *
     * Gibt Zugriff auf eine, für das Plugin passende, implementierung von [IDataAccess](@ref IDataAccess)
     */
    virtual IDataAccess* DataAccess() const = 0;
    /*!
     * \brief getAlgorithm
     * \return Der [Algorithmus](@ref IAlgorithm) des Plugins
     *
     * Gibt einen Zugriff auf den Konkreten [Algorithmus](@ref IAlgorithm) des Plugins
     */
    virtual IAlgorithm* getAlgorithm() const = 0;
    /*!
     * \brief Autor
     * \return Der Autor des Plugins
     *
     * Der Autor des Plugins
     */
    virtual QString Autor() const = 0;
    /*!
     * \brief Date
     * \return Datum der letzten Änderung
     *
     * Datum der letzten Änderung
     */
    virtual QDate Date() const = 0;
    /*!
     * \brief Version
     * \return Versionsnummer des Plugins
     *
     * Versionsnummer des Plugins
     */
    virtual qint32 Version() const = 0;

    // Plugin Types:
    /*!
     * \brief Feature Matcher Plugin
     */
    static const QString PT_FeatureMatcher;
    /*!
     * \brief Tiefenschätzer Plugin
     */
    static const QString PT_DepthMapper;
    /*!
     * \brief Posenschätzung Plugin
     */
    static const QString PT_PoseEstimator;
    /*!
     * \brief PCL Rekonstruktionsplugin
     */
    static const QString PT_PclReconstructor;
};

#endif // APLUGIN_H
