#include "AStreamProvider.h"

AStreamProvider::AStreamProvider() {

}

void AStreamProvider::setDestination(QDir destinationFolder) {
    folder = destinationFolder;
}

QDir AStreamProvider::getDestination() const {
    return fileName;
}

void AStreamProvider::setFileName(const QString &file) {
    this->fileName = file;
}
