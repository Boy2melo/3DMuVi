#include "CImageIo.h"

void CImageIo::save(QImage image, QUrl path) {
    image.save(path.path(), "PNG");
}

QImage CImageIo::load(QUrl path) {
    QImage result;
    result.load(path.path(), "PNG");
    return result;
}
