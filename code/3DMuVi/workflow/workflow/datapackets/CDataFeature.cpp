#include "CDataFeature.h"
#include "workflow/workflow/ccontextdatastore.h"
#include <tuple>

///////////////////////////// Erster Entwurf - Komplett ungetestet /////////////////////

CDataFeature::CDataFeature() {}
CDataFeature::~CDataFeature() {
    if(streamProvider != nullptr){
        delete(streamProvider);
    }
}

QString CDataFeature::getDataType() const {
    return DT_FEATURE_MATCH;
}

void CDataFeature::setFeatureMatch(FeatureMatch && match){
    featureMatchData = match;
}

FeatureMatch const & CDataFeature::getFeatureMatch(){
    return featureMatchData;
}

AStreamProvider* CDataFeature::getStreamProvider() {
    if(streamProvider != nullptr){
        delete(streamProvider);
    }
    streamProvider = new CSFStreamProvider(this->getId());
    return streamProvider;
}

void CDataFeature::serialize(AStreamProvider* stream){
    QDataStream* dataStream = stream->getNextStream();
    uint64_t value0;
    float value1;
    float value2;
    uint32_t value3;
    *dataStream << (int)featureMatchData.size();
    for(std::tuple<uint64_t, float, float, uint32_t> data : featureMatchData){
        std::tie(value0, value1, value2, value3) = data;
        *dataStream << (quint64)value0 << value1 << value2 << (quint32)value3;
    }
}
