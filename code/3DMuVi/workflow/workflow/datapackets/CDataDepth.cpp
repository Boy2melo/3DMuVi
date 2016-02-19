#include "CDataDepth.h"

CDataDepth::CDataDepth()
{

}

CDataDepth::~CDataDepth(){
    if(streamProvider != nullptr){
        delete(streamProvider);
    }
}

QString CDataDepth::getDataType() const{
    return DT_DEPTH;
}

AStreamProvider* CDataDepth::getStreamProvider(){
    if(streamProvider != nullptr){
        delete(streamProvider);
    }
    streamProvider = new CMFStreamProvider;
    return streamProvider;
}

void CDataDepth::serialize(AStreamProvider *stream){
    stream->setFileName(QString("dataDepthMeta"));
    QDataStream* dataStream = stream->getNextStream();
    *dataStream << (int)depthMap.size();

    int fileNameCounter = 0;
    for(QImage map : depthMap){
        stream->setFileName(QString(fileNameCounter + ".png"));
        QDataStream* dataStream = stream->getNextStream();

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

FeatureMatch const & CDataDepth::getFeatureMatch(){
    return featureMatch;
}

void CDataDepth::setFeatureMatch(FeatureMatch && feature){
    featureMatch = feature;
}

FeatureMap const & CDataDepth::getFeatureMap(){
    return featureMap;
}

void CDataDepth::setFeatureMap(FeatureMap && map){
    featureMap = map;
}

std::vector<SPose> const & CDataDepth::getPose(){
    return pose;
}

void CDataDepth::setPose(std::vector<SPose> && poses){
    pose = poses;
}

std::vector<QImage> const & CDataDepth::getDepthMap(){
    return depthMap;
}

void CDataDepth::setDepthMap(std::vector<QImage> &&depthMaps){
    depthMap = depthMaps;
}
