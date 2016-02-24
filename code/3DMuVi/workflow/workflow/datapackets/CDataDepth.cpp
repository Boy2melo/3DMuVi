#include "CDataDepth.h"

CDataDepth::CDataDepth() {
    streamProvider = nullptr;
}

CDataDepth::~CDataDepth() {
    if (streamProvider != nullptr) {
        delete(streamProvider);
    }
}

QString CDataDepth::getDataType() const {
    return DT_DEPTH;
}

AStreamProvider* CDataDepth::getStreamProvider() {
    if (streamProvider != nullptr) {
        delete(streamProvider);
    }
    streamProvider = new CMFStreamProvider;
    return streamProvider;
}

void CDataDepth::serialize(AStreamProvider *stream) {
    stream->setFileName(QString("dataDepthMeta"));
    auto dataStream = stream->getNextStream();
    *dataStream << static_cast<int>(depthMap.size());

    auto fileNameCounter = 0;
    for (auto map : depthMap) {
        stream->setFileName(QString(fileNameCounter + ".png"));
        dataStream = stream->getNextStream();

        //serialize a QImage
        QByteArray byteArray;
        QBuffer buffer(&byteArray);
        buffer.open(QIODevice::WriteOnly);
        map.save(&buffer, "PNG");

        *dataStream << buffer.data();
        buffer.close();

        ++fileNameCounter;
    }
}

std::vector<QImage> const & CDataDepth::getDepthMap() const {
    return depthMap;
}

void CDataDepth::setDepthMap(std::vector<QImage> &&depthMaps) {
    depthMap = depthMaps;
}
