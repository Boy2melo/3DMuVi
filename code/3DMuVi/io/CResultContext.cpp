#include "CResultContext.h"

CResultContext::CResultContext(QUrl path,
                               CLogController logController,
                               CAlgorithmSettingController algoSettings,
                               CGlobalSettingController globalSettings)
{

}

void CResultContext::addDataPacket(IDataPacket data)
{

}

std::vector<QString> CResultContext::getDataPacketIds()
{
    std::vector<QString> list;
    return list;
}

IDataPacket* CResultContext::getDataPacket(QString id)
{
    return nullptr;
}
