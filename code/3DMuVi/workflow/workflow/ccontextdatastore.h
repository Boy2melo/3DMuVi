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
   \class CContextDataStore
 * \brief The CContextDataStore class
 * \author Nathanael Schneider
 *
 * Enthält die Daten, welche für einen Bestimmten [Workflow](@ref AWorkflow) zur Verfügung gestellt werden. Jeder Datentyp kann genau einmal im Store vorkommen. Für mehrere gleiche Daten an unterschiedlichen Schritten sind unterschiedliche Klassen anzulegen.
 */
class EXPORTED CContextDataStore {
private:
    QString mContextId;
    qint32 mCalculationStep;
    bool mAborted;
    QList<std::shared_ptr<IDataPacket>> mDataPackets;
protected:
public:
    ~CContextDataStore();

    /*!
     * \brief Erstelle einen neuen DataStore
     */
    CContextDataStore();

    /*!
     * \brief Initialisiert den Context aus einem Data Store
     */
    void InitializeFromStorage(CInputDataSet *inputData);
    /*!
     * \brief Gibt Daten vom Typ T zurück, sofern vorhanden.
     * \return Daten vom Typ T oder 0 falls nicht vorhanden
     */
    template<typename T>
    std::shared_ptr<T> getData();

    /*!
    \brief Fügt daten vom Typ T hinzu und gibt den Pointer auf das Packet zurück
    \param overwrite Ob die Daten, falls sie existieren, ersetzt werden sollen. Ansonsten wird bei vorhandensein nichts gemacht
    */
    template<typename T>
    std::shared_ptr<T> createData(bool overwrite = true);

    /*!
    \brief Fügt Daten vom Typ T hinzu
    \param data Die Daten auf dem Heap
    \param overwrite Ob die Daten, falls sie existieren, ersetzt werden sollen. Ansonsten wird bei vorhandensein nichts gemacht
    \return True, falls das Packet hinzugefügt wurde
    */
    template<typename T>
    bool appendData(std::shared_ptr<T> data, bool overwrite = true);

    /*!
     * \brief Wendet die Daten des Stores auf einen DataView an.
     */
    void ApplyToDataView(IDataView *view) const;
    /*!
     * \brief Die ID des Datenkontext
     * \return Die ID des Datenkontext
     */
    QString getId() const;
    /*!
     * \brief Schreibe den Context auf die Festplatte
     */
    void Serialize(CResultContext *context);
    /*!
     * \brief Der aktuelle Verarbeitungsschritt im Workflow, in dem der Context sich befindet
     * \return Der aktuelle Verarbeitungsschritt
     */
    qint32 getCurrentCalculationStep() const;

    /*!
    \brief Reset the current calculation step counter*/
    void resetCalculationStep();

    /*!
    \brief Increment the current calculation step
    */
    void incCalculationStep();

    /*!
    \brief Whether this data store should not be passed to the next algorithm
    */
    bool IsAborted() const;

    /*!
    \brief Set whether the data should be passed to the next algorithm
    */
    void SetIsAborted(bool abort);


#define DT_FEATURE_MATCH "FeatureMatch Data"
#define DT_POSE "Pose Data"
#define DT_DEPTH "Depth Data"
#define DT_FUSION "Fusion Data"
#define DT_INPUTIMAGES "Input Images"
};

#endif // ACONTEXTDATASTORE_H
