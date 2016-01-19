#ifndef ACONTEXTDATASTORE_H
#define ACONTEXTDATASTORE_H

#include <QVector>

/*!
 * \brief The AContextDataStore class
 *
 * Enthält die Daten, welche für einen Bestimmten Workflow zur Verfügung gestellt werden. Jeder Datentyp kann genau einmal im Store vorkommen. Für mehrere gleiche Daten an unterschiedlichen Schritten sind unterschiedliche Klassen anzulegen.
 */
class AContextDataStore
{
public:
    AContextDataStore();
    QVector<QString> getSupportedDataTypes();
    void InitializeFromStorage(/*TODO*/);

    template<typename T>
    T* getData();

    void ApplyToDataView(/*TODO*/);
    const QString getId();
    void Serialize();
    qint32 getCurrentCalculationStep();

    // Data Types

};

#endif // ACONTEXTDATASTORE_H
