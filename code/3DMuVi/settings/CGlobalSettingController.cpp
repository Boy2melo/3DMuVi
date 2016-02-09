#include "CGlobalSettingController.h"
#include "QJsonDocument"
#include "io/CTextIo.h"

CGlobalSettingController::CGlobalSettingController()
{
    QUrl directory;
    io = CTextIo();
    directory = QUrl(QString("Verzeichnis"));//verzeichniss fehlt noch
    import(directory, "globalconfig");
}

QString CGlobalSettingController::getSetting(QString name)
{
     QJsonValue value = settings.value(name);
     QString result = QString(value.toString());
     return result;
}

bool CGlobalSettingController::setSetting(QString name, QString value)
{ 
    if(settings.contains(name)){
        settings.insert(name, QJsonValue(value));
        return true;
    } else {
        return false;
    }
}

void CGlobalSettingController::import(QUrl directory, QString name)
{
    QUrl url;
    QString file;
    url = QUrl(directory.toString() + name);
    file = io.load(url);
    QJsonDocument docu = QJsonDocument().fromJson(file.toUtf8());
    settings = docu.object();
}

void CGlobalSettingController::exportto(QUrl directory)
{
    QJsonDocument docu = QJsonDocument(settings);
    QString file = QString(docu.toJson());
    io.save(directory, file);
}

void CGlobalSettingController::resettoDefault()
{
    QUrl directory;
    directory = QUrl(QString("Verzeichnis"));//defaultverzeichniss fehlt noch
    import(directory, "defaultglobalconfig");
}
