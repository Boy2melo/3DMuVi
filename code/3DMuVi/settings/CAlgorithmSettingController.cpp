#include "CAlgorithmSettingController.h"
#include "QJsonDocument"
#include "CQJsonModel.h"


CAlgorithmSettingController::CAlgorithmSettingController(QUrl directory)
{
    tempdirectory = QUrl(directory);
    io = CTextIo();
    //QObject::connect(CQJsonModel, &CQJsonModel::requestQJson(QUrl),
      //               CAlgorithmSettingController, &CAlgorithmSettingController::requestQJson(QUrl));
    //QObject::connect(CQJsonModel, &CQJsonModel::saveQJson(QJsonObject, QUrl),
      //               CAlgorithmSettingController, &CAlgorithmSettingController::saveQJson(QJsonObject, QUrl));
}
QJsonValue CAlgorithmSettingController::getSetting(QString name, QString key)
{
    QUrl url;
    QString file;
    QJsonObject tempjson;
    if (algorithms.contains(name)) {
       url = QUrl(tempdirectory.toString() + name);
       file = io.load(url);
       QJsonDocument docu = QJsonDocument().fromJson(file.toUtf8());
       tempjson = docu.object();
    }
    return tempjson.value(key);
}

bool CAlgorithmSettingController::setSetting(QString name,QString key, QJsonValue value)
{
    QJsonDocument docu;
    QString file;
    QUrl url;
    QJsonObject tempjson;
    if (!algorithms.contains (name)) {
        return false;
    }
    url = QUrl(tempdirectory.toString() + name);
    file = io.load(url);
    docu = QJsonDocument().fromJson(file.toUtf8());
    tempjson = docu.object();
    tempjson.insert(key, value);
    docu = QJsonDocument(tempjson);
    file = QString(docu.toJson());
    url = QUrl(tempdirectory.toString() + name);
    io.save(url, file);
    return true;
}

QJsonObject* CAlgorithmSettingController::getSetting(QString name)
{
    QUrl url;
    QString file;
    QJsonDocument docu;
    if (algorithms.contains(name)) {
       url = QUrl(tempdirectory.toString() + name);
       file = io.load(url);
       docu = QJsonDocument().fromJson(file.toUtf8());
       QJsonObject* object = new QJsonObject(docu.object());
       return object;
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
        io.save(url, file);
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
    file = io.load(url);
    url = QUrl(tempdirectory.toString() + name);
    io.save(url, file);
}

void CAlgorithmSettingController::exportto(QUrl directory)
{
    QString file;
    QUrl url;
    QString name;
    for (int i = 0; i < algorithms.length(); i++) {
        name = algorithms.value(i);
        url = QUrl(tempdirectory.toString() + name);
        file = io.load(url);
        url = QUrl(directory.toString() + name);
        io.save(url, file);
    }
}
void CAlgorithmSettingController::requestQJson(QUrl directory)
{
    QString file = io.load(directory);
    QJsonDocument docu = QJsonDocument().fromJson(file.toUtf8());
    //emit loadQJson(docu.object());
}

void CAlgorithmSettingController::saveQJson(QJsonObject data, QUrl directory)
{
    QJsonDocument docu = QJsonDocument(data);
    QString file = QString(docu.toJson());
    io.save(directory, file);
}
