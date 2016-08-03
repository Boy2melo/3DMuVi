#include "CDataDepth.h"

QString CDataDepth::getDataType() const {
    return DT_DEPTH;
}

std::unique_ptr<AStreamProvider> CDataDepth::getStreamProvider() {
    return std::unique_ptr<AStreamProvider>(new CMFStreamProvider);
}

void CDataDepth::serialize(AStreamProvider *stream) {
    stream->setFileName(QString("dataDepthMeta"));
    auto dataStream = stream->getNextStream();

    uint32_t id;
    QImage dMap;

    if(depthMap) {
        *dataStream << static_cast<int>(depthMap->size());
        for (auto map : *depthMap) {
            std::tie(id, std::ignore) = map;
            *dataStream << static_cast<quint64>(id);
        }

        auto fileNameCounter = 0;
        for (auto map : *depthMap) {
            std::tie(id, dMap) = map;

            stream->setFileName(QString::number(id) + ".png");
            dataStream = stream->getNextStream();

            //serialize a QImage
            QByteArray byteArray;
            QBuffer buffer(&byteArray);
            buffer.open(QIODevice::WriteOnly);
            dMap.save(&buffer, "PNG");

            *dataStream << buffer.data();
            buffer.close();

            ++fileNameCounter;
        }
    } else {
        *dataStream << (int)0;
    }
}

void CDataDepth::deserialize(AStreamProvider *stream) {
    //TODO: implement deserialize
    Q_UNUSED(stream);
}

std::shared_ptr<std::vector<std::tuple<uint32_t, QImage>>> CDataDepth::getDepthMap() const {
    return depthMap;
}

void CDataDepth::setDepthMap(std::shared_ptr<std::vector<std::tuple<uint32_t, QImage>>> depthMaps) {
    depthMap = depthMaps;
}

void CDataDepth::applyToDataview(IDataView* dataView) const
{
  dataView->applyData(shared_from_this());
}
