#ifndef IDATAACCESS_H
#define IDATAACCESS_H

#include <QVector>
#include "workflow/workflow/acontextdatastore.h"

/*!
 * \brief The IDataAccess class
 *
 * Abstrahiert den Zugriff zu den Daten, die von einem bestimmten Plugin benötigt werden und liefert Informationen zu den Verbrauchten und Produzierten Datentypen.
 */
class IDataAccess
{
public:
    IDataAccess();
    /*!
     * \brief getInputDataTypes
     * \return Eine Liste aller Daten, die als Eingabe benötigt werden.
     *
     * Eine Liste aller Daten, die als Eingabe benötigt werden.
     */
    QVector<QString> getInputDataTypes();
    /*!
     * \brief getOutputDataTypes
     * \return Eine Liste aller Daten, die als Ausgabe erzeugt werden.
     *
     * Eine Liste aller Daten, die als Ausgabe erzeugt werden.
     */
    QVector<QString> getOutputDataTypes();
    /*!
     * \brief bindToDataStore
     * \param dataStore Der DataStore für den nächsten Durchlauf
     * Lade die Daten für den Algorithmus von gegebenem DataStore
     */
    void bindToDataStore(AContextDataStore* dataStore);
};

#endif // IDATAACCESS_H
