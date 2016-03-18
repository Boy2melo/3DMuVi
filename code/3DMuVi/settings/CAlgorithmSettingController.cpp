#include "CAlgorithmSettingController.h"
#include "QJsonDocument"
#include "CQJsonModel.h"
#include "logger/controll/CLogController.h"
#include "io/CTextIo.h"

CAlgorithmSettingController::CAlgorithmSettingController(QUrl directory) {
    tempdirectory = QUrl(directory);
    io = CTextIo();

}
QJsonValue CAlgorithmSettingController::getSetting(QString name, QString key) const {
    QUrl url;
    QString file;
    QJsonObject tempjson;
    if (algorithms.contains(name)) {
        url = QUrl(tempdirectory.url() + name + ".json");
        file = io.load(url);
        QJsonDocument docu = QJsonDocument().fromJson(file.toUtf8());
        tempjson = docu.object();
    }
    return tempjson.value(key);
}

bool CAlgorithmSettingController::setSetting(QString name, QString key, QJsonValue value) const {
    QJsonDocument docu;
    QString file;
    QUrl url;
    QJsonObject tempjson;
    if (!algorithms.contains(name)) {
        return false;
    }
    url = QUrl(tempdirectory.url() + name + ".json");
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

QJsonObject* CAlgorithmSettingController::getSetting(QString name) const {
    QUrl url;
    QString file;
    QJsonDocument docu;
    if (algorithms.contains(name)) {
        url = QUrl(tempdirectory.path() + QDir::separator() + name);
        file = io.load(url);
        docu = QJsonDocument().fromJson(file.toUtf8());
        QJsonObject* object = new QJsonObject(docu.object());
        return object;
    }

    return nullptr;
}

bool CAlgorithmSettingController::setSetting(QString name, QJsonObject data) {
    QJsonDocument docu;
    QString file;
    QUrl url;
    if (!algorithms.contains(name)) {
        algorithms.append(name);
    }

    docu = QJsonDocument(data);
    file = QString(docu.toJson());
    url = QUrl(tempdirectory.url() + "/" + name + ".json");
    io.save(url, file);
    return true;
}

void CAlgorithmSettingController::import(QUrl directory, QString name) {
    QString file;
    QUrl url;
    if (!algorithms.contains(name)) {
        algorithms.append(name);
    }
    url = QUrl(directory.toString() + name);
    file = io.load(url);
    url = QUrl(tempdirectory.toString() + name);
    io.save(url, file);
}

void CAlgorithmSettingController::exportTo(QUrl directory) const {
    QString file;
    QUrl url;
    QString name;
    for (int i = 0; i < algorithms.length(); i++) {
        name = algorithms.value(i);
        url = QUrl(tempdirectory.path() + QDir::separator() + name + ".json");
        file = io.load(url);
        url = QUrl(directory.toString() + QDir::separator() + name + ".json");
        io.save(url, file);
    }
}
void CAlgorithmSettingController::requestQJson(QUrl directory) {
    if (directory.password().startsWith("a")) {
        QString temp = directory.password();
        temp.remove(0, 1);
        directory = QUrl(tempdirectory.url() + "/" + temp + ".json");
        if (!algorithms.contains(temp)) {
            algorithms.append(temp);
        }
    } else
    {
        if (!algorithms.contains(directory.fileName())) {
            algorithms.append(directory.fileName());
        }
    }
    QString file = io.load(directory);
    QJsonDocument docu = QJsonDocument().fromJson(file.toUtf8());
    QJsonObject object = docu.object();
    emit loadQJson(object);
}
void CAlgorithmSettingController::saveQJson(QJsonObject data, QUrl directory) {
    if (directory.password().startsWith("a")) {
        QString temp = directory.password();
        temp.remove(0, 1);
        directory = QUrl(tempdirectory.url() + "/" + temp + ".json");
        if (!algorithms.contains(temp)) {
            algorithms.append(temp);
            QJsonDocument docu = QJsonDocument(data);
            QString file = QString(docu.toJson());
            io.save(directory, file);
        }
    }
}
void CAlgorithmSettingController::saveQJsonEx(QJsonObject data, QUrl directory) {

    QJsonDocument docu = QJsonDocument(data);
    QString filetext = QString(docu.toJson());
    QFile file(directory.isLocalFile() ? directory.toLocalFile() : directory.path());
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)){
        CLogController::instance().frameworkMessage("Error: Save Parameter Failed Please Choose Same Disk as Framework");
        return;
    }
    QTextStream out(&file);
    CLogController::instance().frameworkMessage("File: " + directory.toString() + " successfully saved");
    out << filetext;
}
