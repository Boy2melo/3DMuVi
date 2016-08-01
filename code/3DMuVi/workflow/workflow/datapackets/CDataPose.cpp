#include "CDataPose.h"

//Erster Entwurf - Komplett ungetestet.

CDataPose::CDataPose() {
    streamProvider = nullptr;
}

CDataPose::~CDataPose() {
    if (streamProvider != nullptr) {
        delete(streamProvider);
    }
}

QString CDataPose::getDataType() const {
    return DT_POSE;
}

AStreamProvider* CDataPose::getStreamProvider() {
    if (streamProvider != nullptr) {
        delete(streamProvider);
    }
    streamProvider = new CSFStreamProvider();
    return streamProvider;
}

void CDataPose::cleanUpStreamProvider()
{
  if (streamProvider != nullptr) {
      delete(streamProvider);
      streamProvider = nullptr;
  }
}

void CDataPose::serialize(AStreamProvider *stream) {
    stream->setFileName(this->getId());
    auto dataStream = stream->getNextStream();

    //serialize featureMap
    if (featureMap) {
        uint32_t value0;
        float value1;
        float value2;
        float value3;
        float value4;
        *dataStream << static_cast<int>(featureMap->size());
        for (auto data : *featureMap) {
            std::tie(value0, value1, value2, value3, value4) = data;
            *dataStream << static_cast<quint32>(value0) << value1 << value2 << value3 << value4;
        }
    } else {
        *dataStream << static_cast<int>(0);
    }

    //serialize pose
    if (pose) {
        *dataStream << static_cast<int>(pose->size());
        for (auto p : *pose) {
            *dataStream << static_cast<quint64>(p.cameraId) << p.translation << p.orientation << p.principalPoint;

            *dataStream << static_cast<int>(p.focalLength.size());
            for (auto f : p.focalLength) {
                *dataStream << f;
            }

            *dataStream << static_cast<int>(p.distortion.size());
            for (auto d : p.distortion) {
                *dataStream << d;
            }
        }
    } else {
        *dataStream << static_cast<int>(0);
    }
}

void CDataPose::deserialize(AStreamProvider *stream) {
    //TODO: implement deserialize
    Q_UNUSED(stream);
}

std::shared_ptr<FeatureMap> CDataPose::getFeatureMap() const {
    return featureMap;
}

void CDataPose::setFeatureMap(std::shared_ptr<FeatureMap>  map) {
    featureMap = map;
}

std::shared_ptr<std::vector<SPose>> CDataPose::getPose() const {
    return pose;
}

void CDataPose::setPose(std::shared_ptr<std::vector<SPose>> poses) {
    pose = poses;
}

void CDataPose::applyToDataview(IDataView* dataView) const
{
  dataView->applyData(shared_from_this());
}
