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
class IPlugin
{
public:
  /*!
   * \brief Gibt einen Zugriff auf den Konkreten [Algorithmus](@ref IAlgorithm) des Plugins.
   * \return Der [Algorithmus](@ref IAlgorithm) des Plugins.
   */
  virtual IAlgorithm* getAlgorithm() const = 0;

  /*!
   * \brief Initialisiert das Plugin inklusiver seiner Metadaten.
   */
  virtual void Initialize(QPluginLoader* loader) = 0;

  /*!
   * \brief Leerer Destruktor zum Überschreiben durch abgeleitete Klassen.
   */
  virtual ~IPlugin() {}

  /*!
   * \brief Gibt den Autor des Plugins zurück.
   * \return Der Autor des Plugins.
   */
  virtual QString Autor() const = 0;
  /*!
   * \brief Gibt den Namen des Plugins zurück.
   * \return Der Name des Plugins.
   */
  virtual QString Name() const = 0;
  /*!
   * \brief Gibt das Datum der letzten Änderung des Plugins zurück.
   * \return Das Datum der letzten Änderung des Plugins.
   */
  virtual QDate Date() const = 0;
  /*!
   * \brief Gibt die Versionsnummer des Plugins zurück.
   * \return Die Versionsnummer des Plugins.
   */
  virtual qint32 Version() const = 0;

  /*!
   * \brief Gibt den Typ des Plugins zurück.
   * \return Der Typ des Plugins, wie in [CPluginManager](@ref CPluginManager) definiert.
   */
  virtual QString GetPluginType() const = 0;

  /*!
   * \brief Gibt die Parametervoreinstellungen als Json zurück.
   * \return Die Parametervoreinstellungen als Json.
   */
  virtual QJsonObject GetParameterJson() const = 0;

  /*!
   * \brief Gibt die Beschreibung für die einzelnen Parameter zurück.
   * \return Die Beschreibung für die einzelnen Parameter.
   */
  virtual QJsonObject GetParameterDescriptionJson() const = 0;

  /*!
  \brief Prüfe alle Parameter auf gültige Werte.
  \return True, falls alle Werte sich in gültigen Grenzen befinden. False andernfalls.
  */
  virtual bool ValidateParameters(QJsonObject*) const = 0;
};

Q_DECLARE_INTERFACE(IPlugin, IPlugin_iid)

#endif // APLUGIN_H
