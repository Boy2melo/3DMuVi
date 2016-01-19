#ifndef IALGORITHM_H
#define IALGORITHM_H

/*!
 * \brief The IAlgorithm class
 *
 * Ein Interface, dass die Funktionen der Algorithmen verallgemeinert und unabhängig von Typen macht, wodurch das Template TAlgorithm polymorph wird.
 */
class IAlgorithm
{
public:
    IAlgorithm();
    /*!
     * \brief setLogger (Logger Instance)
     * Initialisiert einen Logger für den Algorithmus
     */
    void setLogger(/*TODO &*/);
    /*!
     * \brief setParameters (Parameter Instance)
     *
     * Setze die Parameter für den Algorithmus
     */
    void setParameters(/*TODO &*/);
    /*!
     * \brief run
     *
     * Führe dem Algorithmus auf den dem Plugin bekannten Daten aus.
     */
    void run();
};

#endif // IALGORITHM_H
