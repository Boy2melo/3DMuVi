#ifndef PLUGIN_H
#define PLUGIN_H

#include <QObject>
#include <QJsonObject>
#include "workflow/plugin/iplugin.h"
#include "algorithm.h"
#include "plugin_config.h"

class _CLASS_GEN(Plugin) : public QObject, public IPlugin
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID IPlugin_iid FILE "Plugin.json")
    Q_INTERFACES(IPlugin)

private:
    QString mAutor;
    quint32 mVersion;
    QDate mDate;
    QJsonObject mParameters;

    // TODO: set algorithm type
    _CLASS_GEN(Algorithm)* mAlgorithm;
public:
    _CLASS_GEN(Plugin)();

    virtual ~_CLASS_GEN(Plugin)();
    /*!
     * \brief getAlgorithm
     * \return Der [Algorithmus](@ref IAlgorithm) des Plugins
     *
     * Gibt einen Zugriff auf den Konkreten [Algorithmus](@ref IAlgorithm) des Plugins
     */
    virtual IAlgorithm* getAlgorithm() const override;

    /*!
     * \brief Initialize the Plugin with its metadata
     */
    virtual void Initialize(QPluginLoader *loader) override;
    /*!
     * \brief Autor
     * \return Der Autor des Plugins
     *
     * Der Autor des Plugins
     */
    virtual QString Autor() const override;
    /*!
     * \brief Date
     * \return Datum der letzten Änderung
     *
     * Datum der letzten Änderung
     */
    virtual QDate Date() const override;
    /*!
     * \brief Version
     * \return Versionsnummer des Plugins
     *
     * Versionsnummer des Plugins
     */
    virtual qint32 Version() const override;

    /*!
    \brief Gibt den Typ des Plugins an
    \return Der Typ des Plugins wie in [CPluginManager](@ref CPluginManager) definiert
    */
    virtual QString GetPluginType() const override;

    /*!
    \brief Gibt die Parametervoreinstellungen als Json
    */
    virtual QJsonObject GetParameterJson() const override;

    /*!
     * \brief Gibt die Beschreibung der Parameter als Json
     */
    virtual QJsonObject GetParameterDescriptionJson() const override;

    /*!
    \brief Prüfe alle Parameter auf gültige Werte
    \return True falls alle Werte sich in gültigen Grenzen befinden, False andernfalls
    */
    virtual bool ValidateParameters(QJsonObject*) const override;
};

#endif // PLUGIN_H
