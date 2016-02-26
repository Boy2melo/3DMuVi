#include "CSFStreamProvider.h"

CSFStreamProvider::CSFStreamProvider(){
    stream = nullptr;
    file = nullptr;
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
        file = new QFile(folder.absoluteFilePath(fileName));
        if(!file->open(QIODevice::WriteOnly))
            return nullptr;
        stream = new QDataStream(file);
    }
    return stream;
}
