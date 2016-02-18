#include "CDataPose.h"

//Erster Entwurf - Komplett ungetestet.

CDataPose::CDataPose(){

}

CDataPose::~CDataPose(){
    if(streamProvider != nullptr){
        delete(streamProvider);
    }
}

QString CDataPose::getDataType() const{
    return DT_POSE;
}

AStreamProvider* CDataPose::getStreamProvider(){
    if(streamProvider != nullptr){
        delete(streamProvider);
    }
    streamProvider = new CSFStreamProvider(this->getId());
    return streamProvider;
}

void CDataPose::serialize(AStreamProvider *stream){
    QDataStream* dataStream = stream->getNextStream();

    //serialize featureMap
    uint32_t value0;
    float value1;
    float value2;
    float value3;
    float value4;
    *dataStream << (int)featureMap.size();
    for(std::tuple<uint32_t, float, float, float, float> data : featureMap){
        std::tie(value0, value1, value2, value3, value4) = data;
        *dataStream << (quint32)value0 << value1 << value2 << value3 << value4;
    }

    //serialize pose
    *dataStream << (int)pose.size();
    for(SPose p : pose){
        *dataStream << (quint64)p.cameraId << p.translation << p.orientation << p.principalPoint;

        *dataStream << (int)p.focalLength.size();
        for(float f : p.focalLength){
            *dataStream <<  f;
        }

        *dataStream << (int)p.distortion.size();
        for(float d : p.distortion){
            *dataStream << d;
        }
    }
}

FeatureMatch const & CDataPose::getFeatureMatch(){
    return featureMatch;
}

void CDataPose::setFeatureMatch(FeatureMatch && feature){
    featureMatch = feature;
}

FeatureMap const & CDataPose::getFeatureMap(){
    return featureMap;
}

void CDataPose::setFeatureMap(FeatureMap && map){
    featureMap = map;
}

std::vector<SPose> const & getPose(){
    //TODO
    return std::vector<SPose>();
}

void CDataPose::setPose(std::vector<SPose> && poses){
    pose = poses;
}
