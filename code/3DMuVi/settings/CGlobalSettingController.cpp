#include "CGlobalSettingController.h"
#include <QJsonDocument>
#include "io/CTextIo.h"
#include <QFileInfo>
#include <QDir>

CGlobalSettingController::CGlobalSettingController() {
    QUrl directory;
    io = CTextIo();
    directory = QUrl(QUrl::fromLocalFile(QFileInfo("globalconfig.json").absoluteFilePath()));
    import(directory, directory.fileName());
}

QString CGlobalSettingController::getSetting(QString name) const{
    QJsonValue value = settings.value(name);
    QString result = QString(value.toString());
    return result;
}

bool CGlobalSettingController::setSetting(QString name, QString value) {
    if (settings.contains(name)) {
        settings.insert(name, QJsonValue(value));
        QJsonDocument docu = QJsonDocument(settings);
        QString file = QString(docu.toJson());
        QUrl directory = QUrl(QUrl::fromLocalFile(QFileInfo("globalconfig.json").absoluteFilePath()));
        io.save(directory, file);
        return true;
    } else {
        return false;
    }
}

void CGlobalSettingController::import(QUrl directory, QString name) {
    QUrl url;
    QString file;
    if (directory.fileName() != name) {
        url = QUrl(directory.path() + QDir::separator() + name + ".json");
        directory = url;
    }
    file = io.load(directory);
    QJsonDocument docu = QJsonDocument().fromJson(file.toUtf8());
    settings = docu.object();
}

void CGlobalSettingController::exportTo(QUrl directory) const{
    QJsonDocument docu = QJsonDocument(settings);
    QString file = QString(docu.toJson());
    QUrl url = QUrl(directory.path() + QDir::separator() + "globalconfig.json");
    io.save(url, file);
}

void CGlobalSettingController::resetToDefault() {
    QUrl directory;
    directory = QUrl(QUrl::fromLocalFile(QFileInfo("defaultglobalconfig.json").absoluteFilePath()));
    import(directory, directory.fileName());
}
