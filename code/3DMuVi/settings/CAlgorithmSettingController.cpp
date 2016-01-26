#include "CAlgorithmSettingController.h"
#include "QJsonDocument"

CAlgorithmSettingController::CAlgorithmSettingController(QUrl directory)
{
    tempdirectory = QUrl(directory);
}
QJsonObject CAlgorithmSettingController::getSetting(QString name)
{
    QUrl url;
    QString file;
    if (algorithms.contains(name)) {
       url = QUrl(tempdirectory.toString() + name);
       //file = io.TextIo.load(url);
       QJsonDocument docu = QJsonDocument().fromJson(file.toUtf8());
       return docu.object();
    }
}

bool CAlgorithmSettingController::setSetting(QString name, QJsonObject data)
{
    QJsonDocument docu;
    QString file;
    QUrl url;
    if (!algorithms.contains (name)) {
        algorithms.append(name);
    }
    if(true){//Algorithmen checks if parameters are legal
        docu = QJsonDocument(data);
        file = QString(docu.toJson());
        url = QUrl(tempdirectory.toString() + name);
        //io.TextIo.save(url, file);
        return true;
    } else {
        return false;
    }
}

void CAlgorithmSettingController::import(QUrl directory, QString name)
{ 
    QString file;
    QUrl url;
    if (!algorithms.contains (name)) {
        algorithms.append(name);
    }
    url = QUrl(directory.toString() + name);
    //file = io.TextIo.load(url);
    url = QUrl(tempdirectory.toString() + name);
    //io.TextIo.save(url, file);
}

void CAlgorithmSettingController::exportto(QUrl directory)
{
    QString file;
    QUrl url;
    QString name;
    for (int i = 0; i < algorithms.length(); i++) {
        name = algorithms.value(i);
        url = QUrl(tempdirectory.toString() + name);
        //file = io.TextIo.load(url);
        url = QUrl(directory.toString() + name);
        //io.TextIo.save(url, file);
    }
}
