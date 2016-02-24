#include "CMFStreamProvider.h"

CMFStreamProvider::CMFStreamProvider() {
    stream = nullptr;
    file = nullptr;
}

CMFStreamProvider::~CMFStreamProvider() {
    if (file != nullptr) {
        file->close();
    }
    delete(file);
    delete(stream);
}

QDataStream* CMFStreamProvider::getNextStream() {
    if (file != nullptr) {
        file->close();
        delete(file);
        delete(stream);
    }
    file = new QFile(folder.absoluteFilePath(fileName));
    if (!file->open(QIODevice::WriteOnly))
        return nullptr;
    stream = new QDataStream(file);
    return stream;
}
