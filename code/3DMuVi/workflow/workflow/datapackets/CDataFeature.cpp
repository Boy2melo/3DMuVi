#include "CDataFeature.h"
#include "workflow/workflow/ccontextdatastore.h"
#include <tuple>

///////////////////////////// Erster Entwurf - Komplett ungetestet /////////////////////

CDataFeature::CDataFeature() {
    streamProvider = nullptr;
}

CDataFeature::~CDataFeature() {
    if (streamProvider != nullptr) {
        delete(streamProvider);
    }
}

QString CDataFeature::getDataType() const {
    return DT_FEATURE_MATCH;
}

void CDataFeature::setFeatureMatch(std::shared_ptr<FeatureMatch> match) {
    featureMatchData = match;
}

std::shared_ptr<FeatureMatch> CDataFeature::getFeatureMatch() const {
  return featureMatchData;
}

void CDataFeature::applyToDataview(IDataView* dataView) const
{
  dataView->applyData(shared_from_this());
}

AStreamProvider* CDataFeature::getStreamProvider() {
    if (streamProvider != nullptr) {
        delete(streamProvider);
    }
    streamProvider = new CSFStreamProvider();
    return streamProvider;
}

void CDataFeature::cleanUpStreamProvider()
{
    if (streamProvider != nullptr) {
        delete(streamProvider);
        streamProvider = nullptr;
    }
}

void CDataFeature::serialize(AStreamProvider* stream) {
    stream->setFileName(this->getId());
    auto dataStream = stream->getNextStream();

    if (featureMatchData) {
        uint64_t value0;
        float value1;
        float value2;
        uint32_t value3;
        *dataStream << (int)featureMatchData->size();
        for (auto data : *featureMatchData) {
            std::tie(value0, value1, value2, value3) = data;
            *dataStream << static_cast<quint64>(value0) << value1 << value2 << static_cast<quint32>(value3);
        }
    } else {
        *dataStream << (int)0;
    }
}

void CDataFeature::deserialize(AStreamProvider *stream) {
    //TODO: implement deserialize
    Q_UNUSED(stream);
}
