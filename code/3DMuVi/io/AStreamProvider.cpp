#include "AStreamProvider.h"

AStreamProvider::AStreamProvider()
{

}

void AStreamProvider::setDestination(QDir destinationFolder){
    folder = destinationFolder;
}

void AStreamProvider::setFileName(const QString &fileName){
    this->fileName = fileName;
}
