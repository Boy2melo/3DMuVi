#ifndef ACONTEXTDATASTORE_H
#define ACONTEXTDATASTORE_H

#include <QVector>
#include <QString>
#include "idataview.h"

/*!
   \class AContextDataStore
 * \brief The AContextDataStore class
 * \author Nathanael Schneider
 *
 * Enthält die Daten, welche für einen Bestimmten [Workflow](@ref AWorkflow) zur Verfügung gestellt werden. Jeder Datentyp kann genau einmal im Store vorkommen. Für mehrere gleiche Daten an unterschiedlichen Schritten sind unterschiedliche Klassen anzulegen.
 */
class AContextDataStore
{
private:
    QString mContextId;
    qint32 mCalculationStep;
protected:
    /*!
     * \brief Wird bei der Serialisierung aufgerufen
     * \param context Der Serialisierungskontext
     */
    virtual void OnSerialize(/*TODO*/) = 0;
public:
    virtual ~AContextDataStore() {
    }

    /*!
     * \brief Erstelle einen neuen DataStore
     */
    AContextDataStore();
    /*!
     * \brief Eine Liste an unterstützten Datentypen für diesen Store
     * \return Eine Liste an unterstützten Datentypen für diesen Store
     */
    virtual QVector<QString> getSupportedDataTypes() const = 0;
    /*!
     * \brief Initialisiert den Context aus einem Data Store
     */
    virtual void InitializeFromStorage(/*TODO*/) = 0;
    /*!
     * \brief Gibt Daten vom Typ T zurück, sofern vorhanden.
     * \return Daten vom Typ T oder 0 falls nicht vorhanden
     */
    template<typename T>
    T* getData();
    /*!
     * \brief Wendet die Daten des Stores auf einen DataView an.
     */
    virtual void ApplyToDataView(IDataView *view) const = 0;
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

    // Data Types

};

#endif // ACONTEXTDATASTORE_H
