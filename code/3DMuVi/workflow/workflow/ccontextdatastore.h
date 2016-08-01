#ifndef CCONTEXTDATASTORE_H
#define CCONTEXTDATASTORE_H

#include <QVector>
#include <QString>
#include <QStringList>
#include "idataview.h"
#include "idatapacket.h"
#include <memory>
#include "io/CInputDataSet.h"
#include "io/CResultContext.h"
#include <macros.h>

/*!
 * \class CContextDataStore
 * \brief The CContextDataStore class
 * \author Nathanael Schneider
 *
 * Enthält die Daten, welche für einen bestimmten [Workflow](@ref AWorkflow) zur Verfügung gestellt
 * werden. Jeder Datentyp kann genau einmal im Store vorkommen. Für mehrere gleiche Daten an
 * unterschiedlichen Schritten sind unterschiedliche Klassen anzulegen.
 */
class EXPORTED CContextDataStore
{
public:
  /*!
   * \brief Leerer Destruktor zum Überschreiben durch abgeleitete Klassen.
   */
  ~CContextDataStore();

  /*!
   * \brief Erstelle einen neuen Datastore.
   */
  CContextDataStore();

  /*!
   * \brief Initialisiert den Context aus einem Datastore.
   * \param inputData Die Eingabebilder, die in den Context übernommen werden sollen.
   */
  void initializeFromStorage(CInputDataSet* inputData);
  /*!
   * \brief Gibt Daten vom Typ T zurück, sofern vorhanden.
   * \return Das Datenpacket vom Typ T oder nullptr, falls nicht vorhanden.
   */
  template<typename T>
  std::shared_ptr<T> getData();

  /*!
   * \brief Fügt Daten vom Typ T hinzu und gibt den Pointer auf das Packet zurück.
   * \param overwrite Ob die Daten, falls sie existieren, ersetzt werden sollen. Ansonsten wird bei
   * Vorhandensein nichts gemacht.
   * \return Pointer auf das erstellte Datenpacket.
   */
  template<typename T>
  std::shared_ptr<T> createData(bool overwrite = true);

  /*!
   * \brief Fügt Daten vom Typ T hinzu.
   * \param data Die Daten auf dem Heap.
   * \param overwrite Ob die Daten, falls sie existieren, ersetzt werden sollen. Ansonsten wird bei
   * Vorhandensein nichts gemacht.
   * \return True, falls das Packet hinzugefügt wurde. False andernfalls.
   */
  template<typename T>
  bool appendData(std::shared_ptr<T> data, bool overwrite = true);

  /*!
   * \brief Wendet die Daten des Stores auf einen DataView an.
   * \param view Die View, auf welche die Daten angewendet werden sollen.
   */
  void applyToDataView(IDataView* view) const;
  /*!
   * \brief Gibt die ID des Datenkontext zurück.
   * \return Die ID des Datenkontext.
   */
  QString getId() const;
  /*!
   * \brief Schreibe den Context auf die Festplatte.
   * \param context Der Resultcontext, in den die Datenpackete geschrieben werden sollen.
   */
  void serialize(CResultContext* context);
  /*!
   * \brief Gibt den aktuellen Verarbeitungsschritt im Workflow zurück.
   * \return Der aktuelle Verarbeitungsschritt, in dem der Context sich befindet.
   */
  qint32 getCurrentCalculationStep() const;

  /*!
   * \brief Setzt den aktuellen Verarbeitungsschritt zurück.
   */
  void resetCalculationStep();

  /*!
   * \brief Erhöht den aktuellen Verarbeitungsschritt um eins.
   */
  void incCalculationStep();

  /*!
   * \brief Gibt zurück, ob die Ausführung abgebrochen werden soll.
   * \return True, wenn der nächste Algorithmus nicht mehr ausgeführt werden soll. False
   * andernfalls.
   */
  bool isAborted() const;

  /*!
   * \brief Bricht die Ausführung vor dem nächsten Algorithmus ab.
   * \param abort True, falls der nächste Algorithmus nicht mehr ausgeführt werden soll. False
   * andernfalls.
   */
  void setIsAborted(bool abort);

private:
  QString mContextId;
  qint32 mCalculationStep;
  bool mAborted;
  QList<std::shared_ptr<IDataPacket>> mDataPackets;

#define DT_FEATURE_MATCH "FeatureMatch Data"
#define DT_POSE "Pose Data"
#define DT_DEPTH "Depth Data"
#define DT_FUSION "Fusion Data"
#define DT_INPUTIMAGES "Input Images"
};

#endif // ACONTEXTDATASTORE_H
