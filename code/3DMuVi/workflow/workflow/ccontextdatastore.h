#ifndef CCONTEXTDATASTORE_H
#define CCONTEXTDATASTORE_H

#include <QVector>
#include <QString>
#include <QStringList>
#include "idataview.h"
#include "idatapacket.h"

/*!
   \class CContextDataStore
 * \brief The CContextDataStore class
 * \author Nathanael Schneider
 *
 * Enthält die Daten, welche für einen Bestimmten [Workflow](@ref AWorkflow) zur Verfügung gestellt werden. Jeder Datentyp kann genau einmal im Store vorkommen. Für mehrere gleiche Daten an unterschiedlichen Schritten sind unterschiedliche Klassen anzulegen.
 */
class CContextDataStore {
private:
    QString mContextId;
    qint32 mCalculationStep;
    bool mAborted;
    QList<IDataPacket*> mDataPackets;
protected:
    /*!
     * \brief Wird bei der Serialisierung aufgerufen
     * \param context Der Serialisierungskontext
     */
    void OnSerialize(/*TODO*/);
public:
    ~CContextDataStore();

    /*!
     * \brief Erstelle einen neuen DataStore
     */
    CContextDataStore();

    /*!
     * \brief Initialisiert den Context aus einem Data Store
     */
    void InitializeFromStorage(/*TODO*/);
    /*!
     * \brief Gibt Daten vom Typ T zurück, sofern vorhanden.
     * \return Daten vom Typ T oder 0 falls nicht vorhanden
     */
    template<typename T>
    T* getData();

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
    void Serialize();
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
