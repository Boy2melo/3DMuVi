#ifndef IDATAACCESS_H
#define IDATAACCESS_H

#include <QVector>
#include "workflow/workflow/acontextdatastore.h"

/*!
   \class IDataAccess
 * \brief The IDataAccess class
 * \author Nathanael Schneider
 *
 * Abstrahiert den Zugriff zu den Daten, die von einem bestimmten [Plugin](@ref APlugin) benötigt werden und liefert Informationen zu den Verbrauchten und Produzierten Datentypen.
 */
class IDataAccess
{
public:
    IDataAccess();
    /*!
     * \brief Eine Liste aller Daten, die als Eingabe benötigt werden.
     * \return Eine Liste aller Daten, die als Eingabe benötigt werden.
     */
    virtual QVector<QString> getInputDataTypes() const = 0;
    /*!
     * \brief Eine Liste aller Daten, die als Ausgabe erzeugt werden.
     * \return Eine Liste aller Daten, die als Ausgabe erzeugt werden.
     */
    virtual QVector<QString> getOutputDataTypes() const = 0;
    /*!
     * \brief Lade die Daten für den Algorithmus von gegebenem DataStore
     * \param dataStore Der DataStore für den nächsten Durchlauf
     */
    virtual void bindToDataStore(AContextDataStore* dataStore) = 0;
};

#endif // IDATAACCESS_H
