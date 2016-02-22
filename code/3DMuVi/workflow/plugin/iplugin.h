#ifndef APLUGIN_H
#define APLUGIN_H

#include "ialgorithm.h"
#include <QDate>
#include <QJsonObject>
#include <QtPlugin>
#include <QPluginLoader>

#define IPlugin_iid "org.qt-project.Qt.Fraunhofer.3DMuVi.IPlugin"
/*!
 * \class APlugin
 * \brief The APlugin class
 * \author Nathanael Schneider
 *
 * Beschreibt ein einzelnes Plugin, welches einen einzelnen [Algorithmus](@ref IAlgorithm) kapselt.
 */
class IPlugin {
public:
    /*!
     * \brief getAlgorithm
     * \return Der [Algorithmus](@ref IAlgorithm) des Plugins
     *
     * Gibt einen Zugriff auf den Konkreten [Algorithmus](@ref IAlgorithm) des Plugins
     */
    virtual IAlgorithm* getAlgorithm() const = 0;

    /*!
     * \brief Initialize the Plugin with its metadata
     */
    virtual void Initialize(QPluginLoader *loader) = 0;

    virtual ~IPlugin() {}
    /*!
     * \brief Autor
     * \return Der Autor des Plugins
     *
     * Der Autor des Plugins
     */
    virtual QString Autor() const = 0;

    /*!
    \brief Der Name des Plugins
    */
    virtual QString Name() const = 0;
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

    /*!
    \brief Gibt den Typ des Plugins an
    \return Der Typ des Plugins wie in [CPluginManager](@ref CPluginManager) definiert
    */
    virtual QString GetPluginType() const = 0;

    /*!
    \brief Gibt die Parametervoreinstellungen als Json
    */
    virtual QJsonObject GetParameterJson() const = 0;

    /*!
     * \brief Gibt die Beschreibung für die einzelnen Parameter
     */
    virtual QJsonObject GetParameterDescriptionJson() const = 0;

    /*!
    \brief Prüfe alle Parameter auf gültige Werte
    \return True falls alle Werte sich in gültigen Grenzen befinden, False andernfalls
    */
    virtual bool ValidateParameters(QJsonObject*) const = 0;
};

Q_DECLARE_INTERFACE(IPlugin, IPlugin_iid)
#endif // APLUGIN_H
