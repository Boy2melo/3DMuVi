#include "CSFStreamProvider.h"

CSFStreamProvider::CSFStreamProvider(const QString &fileName){
    name = fileName;
}

CSFStreamProvider::~CSFStreamProvider(){
    file->close();
    delete(file);
    delete(stream);
}

QDataStream* CSFStreamProvider::getNextStream(){
    if(stream == nullptr){
        file = new QFile(folder.absoluteFilePath(name));
        file->open(QIODevice::WriteOnly);
        stream = new QDataStream(file);
    }
    return stream;
}
