#ifndef CMFSTREAMPROVIDER_H
#define CMFSTREAMPROVIDER_H

#include <io/AStreamProvider.h>

/*!
 * \class CMFStreamProvider
 * \brief The ConcreteMultiFileStreamProvider class
 * \author Laurenz Thiel
 *
 * Provide streams to multiple files.
 */
class CMFStreamProvider : public AStreamProvider
{
public:
    CMFStreamProvider();
    ~CMFStreamProvider();
    /*!
     * \brief Everytime this function get called it return a pointer to a new file.
     *        The Filename will be "[i].png" with i starting at 0 and i will incerement for each file.
     *\return a QDataStream pointer to a file.
     */
    QDataStream* getNextStream();
};

#endif // CMFSTREAMPROVIDER_H
