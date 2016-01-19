#ifndef APLUGIN_H
#define APLUGIN_H

#include "idataaccess.h"
#include "ialgorithm.h"
#include <QDate>

/*!
 * \brief The APlugin class
 *
 * Beschreibt ein einzelnes Plugin, welches einen einzelnen Algorithmus kapselt.
 */
class APlugin
{    
public:
    APlugin();
    /*!
     * \brief DataAccess
     * \return Instanz auf eine Implementierung von IDataAccess
     *
     * Gibt Zugriff auf eine, für das Plugin passende, implementierung von IDataAccess
     */
    const IDataAccess* DataAccess();
    /*!
     * \brief getAlgorithm
     * \return Der Algorithmus des Plugins
     *
     * Gibt einen Zugriff auf den Konkreten Algorithmus des Plugins
     */
    const IAlgorithm* getAlgorithm();
    /*!
     * \brief Autor
     * \return Der Autor des Plugins
     *
     * Der Autor des Plugins
     */
    const QString Autor();
    /*!
     * \brief Date
     * \return Datum der letzten Änderung
     *
     * Datum der letzten Änderung
     */
    const QDate Date();
    /*!
     * \brief Version
     * \return Versionsnummer des Plugins
     *
     * Versionsnummer des Plugins
     */
    const qint32 Version();

    // Plugin Types:
    const QString PT_FeatureMatcher = "Feature Matcher";
    const QString PT_DepthMapper = "Depth Mapper";
    const QString PT_PoseEstimator = "Pose Estimator";
    const QString PT_PclReconstructor = "PCL Reconstructor";
};

#endif // APLUGIN_H
