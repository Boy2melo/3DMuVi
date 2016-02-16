#include "CSFStreamProvider.h"

CSFStreamProvider::CSFStreamProvider(const QString& fileName){
    name = fileName;
}

CSFStreamProvider::~CSFStreamProvider(){
    if(file != nullptr){
       file->close();
    }
    delete(file);
    delete(stream);
}

QDataStream* CSFStreamProvider::getNextStream(){
    if(stream == nullptr){
        file = new QFile(folder.absoluteFilePath(name));
        if(!file->open(QIODevice::WriteOnly))
            return nullptr;
        stream = new QDataStream(file);
    }
    return stream;
}
