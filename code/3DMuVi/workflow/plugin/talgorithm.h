#ifndef TALGORITHM_H
#define TALGORITHM_H

#include "ialgorithm.h"

/*!
 * Die Konkrete Implementierung eines Algorithmus.
 */
template<typename T>
class TAlgorithm : IAlgorithm
{
public:
    TAlgorithm();
    /*!
     * \brief setData
     * \param data Der Datencontainer für den Algorithmus
     *
     * Setze die Daten, die der Algorithmus benutzen soll.
     */
    void setData(T* data);
};

#endif // TALGORITHM_H
