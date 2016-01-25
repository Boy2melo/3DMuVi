#ifndef CALGORITHMSETTINGCONTROLLER_H
#define CALGORITHMSETTINGCONTROLLER_H

#include <QString>
#include <QJsonObject>
#include <QUrl>

class CAlgorithmSettingController 
{
public:
  CAlgorithmSettingController(QUrl diretory);
  
  QJsonObject getSetting(QString name);

  bool setSetting(QString name, QJsonObject data);

  void import(QUrl diretory, QString name);

  void exportto(QUrl diretory);
};

#endif //CALGORITHMSETTINGCONTROLLER_H
