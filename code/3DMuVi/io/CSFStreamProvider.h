#ifndef CSFSTREAMPROVIDER_H
#define CSFSTREAMPROVIDER_H

#include <io/AStreamProvider.h>

/*!
 * \class CSFStreamProvider
 * \brief The ConcreteSingleFileStreamProvider class
 * \author Laurenz Thiel
 *
 * Provides a stream to a single file.
 */
class CSFStreamProvider : public AStreamProvider
{
public:
    CSFStreamProvider(const QString& fileName);
    ~CSFStreamProvider();
    /*!
     *\brief Everytime this function get called it return a pointer to the same file.
     *\return a QDataStream pointer to a file.
     */
    QDataStream* getNextStream();
private:
    QDataStream* stream;
    QFile* file;
    QString name;
};

#endif // CSFSTREAMPROVIDER_H
