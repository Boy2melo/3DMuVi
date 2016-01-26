#ifndef CGLOBALSETTINGCONTROLLER_H
#define CGLOBALSETTINGCONTROLLER_H

#include <QString>
#include <QUrl>
#include <QJsonObject>

class CGlobalSettingController
{
private:
    QJsonObject settings;
public:
    /*!
     * constructor for GlobalSettingController
     * /
    CGlobalSettingController();
    /*!
     * gets a certain global setting
     * @param name name of the setting
     * @return value of teh setting as QString
     * /
    QString getSetting(QString name);
    /*!
     * sets a global setting
     * @param name mane of the setting
     * @param data value of the setting
     * /
    bool setSetting(QString name, QString data);
    /*!
     * imports a globalconfig file from a directory, file must be in Json Format
     * @param directory directoy of the file
     * @param name name of the file
     * /
    void import(QUrl diretory, QString name);
    /*!
     * exports the globalsettings in a config file in JsonFormat
     * @param directory the directory where the config should be
     * /
    void exportto(QUrl diretory);
    /*!
     * resets all settings to default
     * /
    void resettoDefault();
};
#endif //CGLOBALSETTINGCONTROLLER_H
