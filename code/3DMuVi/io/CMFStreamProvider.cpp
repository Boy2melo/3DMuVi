#include "CMFStreamProvider.h"

CMFStreamProvider::CMFStreamProvider()
{
}

CMFStreamProvider::~CMFStreamProvider()
{
    if(file != nullptr){
       file->close();
    }
    delete(file);
    delete(stream);
}

QDataStream* CMFStreamProvider::getNextStream(const QString &fileName)
{
    if(file != nullptr){
        file->close();
        delete(file);
        delete(stream);
    }
    file = new QFile(folder.absoluteFilePath(fileName));
    if(!file->open(QIODevice::WriteOnly))
        return nullptr;
    stream = new QDataStream(file);
    return stream;
}
