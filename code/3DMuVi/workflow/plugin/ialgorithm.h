#ifndef IALGORITHM_H
#define IALGORITHM_H

#include <QStringList>
#include <QObject>
#include <QJsonObject>
#include "workflow/workflow/ccontextdatastore.h"
#include "logger/controll/CLogController.h"
#include "settings/CAlgorithmSettingController.h"
#include <functional>

/*!
 * \class IAlgorithm
 * \brief The IAlgorithm class
 * \author Nathanael Schneider
 *
 * Ein Interface, dass die Funktionen der Algorithmen verallgemeinert und unabhängig von Typen
 * macht, wodurch das Template [TAlgorithm](@ref TAlgorithm) polymorph wird.
 */
class IAlgorithm
{
public:
  /*!
   * \brief Leerer Destruktor zum Überschreiben durch abgeleitete Klassen.
   */
  virtual ~IAlgorithm() { }
  /*!
   * \brief Initialisiert einen Logger für den Algorithmus.
   */
  virtual void setLogger(CLogController* controller) = 0;
  /*!
   * \brief Setze die Parameter für den nächsten Durchlauf.
   */
  virtual void setParameters(QJsonObject settings) = 0;
  /*!
   * \brief Führe dem Algorithmus auf den dem Plugin bekannten Daten aus.
   */
  virtual void run(CContextDataStore* dataStore,
                   std::function<void (CContextDataStore*)> callback) = 0;

  /*!
   * \brief Gibt zurück, ob der Algorithmus zur Zeit mit einer Ausführung beschäftigt ist.
   * \return true, falls der Algorithmus beschäftigt ist. false sonst.
   */
  virtual bool IsBusy() const = 0;

  /*!
  * \brief Gibt eine Liste aller Daten, die als Eingabe benötigt werden zurück.
  * \return Eine Liste aller Daten, die als Eingabe benötigt werden.
  */
  virtual QStringList getInputDataTypes() const = 0;
  /*!
  * \brief Gibt eine Liste aller Daten, die als Ausgabe erzeugt werden zurück.
  * \return Eine Liste aller Daten, die als Ausgabe erzeugt werden.
  */
  virtual QStringList getOutputDataTypes() const = 0;
};

#endif // IALGORITHM_H
