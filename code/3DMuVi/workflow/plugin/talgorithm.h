#ifndef TALGORITHM_H
#define TALGORITHM_H

#include "ialgorithm.h"

/*!
   \class TAlgorithm
 * \brief Die Konkrete Implementierung eines [Algorithmus](@ref IAlgorithm).
 * \author Nathanael Schneider
 */
template<typename T>
class TAlgorithm : IAlgorithm
{
protected:
    T* mData;

public:
    /*!
     * \brief Setze die Daten, die der Algorithmus benutzen soll.
     * \param data Der Datencontainer für den Algorithmus
     */
    void setData(T* data);
};

#endif // TALGORITHM_H
