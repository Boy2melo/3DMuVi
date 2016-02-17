#include "CResultContext.h"

CResultContext::CResultContext(QUrl path,
                               CLogController* logController,
                               CAlgorithmSettingController* algoSettings,
                               CGlobalSettingController* globalSettings)
{
    folder = QDir(path.path());
    QString timeString = QDateTime::currentDateTime().toString();
    folder.mkdir(timeString);
    folder.cd(timeString);

    QUrl folderUrl(folder.path());
    logController->setLog(folderUrl);
    algoSettings->exportto(folderUrl);
    globalSettings->exportto(folderUrl);
}

void CResultContext::addDataPacket(IDataPacket* data)
{
    QString dataType = data->getDataType();
    if(!folder.cd(dataType)){
        folder.mkdir(dataType);
        folder.cd(dataType);
    }

    AStreamProvider* streamProvider = data->getStreamProvider();
    streamProvider->setDestination(folder);
    data->serialize(streamProvider);
    delete(streamProvider);

    folder.cdUp();
}

std::vector<QString> CResultContext::getDataPacketIds()
{
    std::vector<QString> list;
    QStringList resultFolders = folder.entryList(QDir::AllDirs);
    for(QString resultFolder : resultFolders){
        folder.cd(resultFolder);
        QStringList resultFiles = folder.entryList(QDir::Files);
        for(QString resultFile : resultFiles){
            list.push_back(resultFile);
        }
        folder.cdUp();
    }
    return list;
}

IDataPacket* CResultContext::getDataPacket(QString id)
{
    return nullptr;
}
