#ifndef IALGORITHM_H
#define IALGORITHM_H

#include <QVector>
#include "workflow/workflow/acontextdatastore.h"

/*!
   \class IAlgorithm
 * \brief The IAlgorithm class
 * \author Nathanael Schneider
 *
 * Ein Interface, dass die Funktionen der Algorithmen verallgemeinert und unabhängig von Typen macht, wodurch das Template [TAlgorithm](@ref TAlgorithm) polymorph wird.
 */
class IAlgorithm
{
public:
    IAlgorithm();
    /*!
     * \brief Initialisiert einen Logger für den Algorithmus
     */
    void setLogger(/*TODO &*/);
    /*!
     * \brief Setze die Parameter für den nächsten Durchlauf
     */
    void setParameters(/*TODO &*/);
    /*!
     * \brief Führe dem Algorithmus auf den dem Plugin bekannten Daten aus.
     */
    virtual void run(AContextDataStore* dataStore) = 0;

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
};

#endif // IALGORITHM_H
