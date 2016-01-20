#ifndef IALGORITHM_H
#define IALGORITHM_H

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
    virtual void run() = 0;
};

#endif // IALGORITHM_H
