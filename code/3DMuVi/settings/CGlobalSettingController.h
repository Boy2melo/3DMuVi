#ifndef CGLOBALSETTINGCONTROLLER_H
#define CGLOBALSETTINGCONTROLLER_H

#include <QString>
#include<QUrl>

class CGlobalSettingController
{
public:
    CGlobalSettingController();

    QString getSetting(QString name);

    bool setSetting(QString name, QString data);

    void import(QUrl diretory, QString name);

    void exportto(QUrl diretory);

    void resettoDefault();
};
#endif //CGLOBALSETTINGCONTROLLER_H
