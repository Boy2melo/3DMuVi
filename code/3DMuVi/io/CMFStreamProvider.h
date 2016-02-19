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
     *  The QDataStream from last call will be deleted.
     * \param fileName of the file.
     * \return a QDataStream pointer to a file.
     */
    QDataStream* getNextStream();

private:
    QDataStream* stream;
    QFile* file;
};

#endif // CMFSTREAMPROVIDER_H
