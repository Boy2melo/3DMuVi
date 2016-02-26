#ifndef CALGORITHMSETTINGSSAVELOADWIDGET_H
#define CALGORITHMSETTINGSSAVELOADWIDGET_H

#include <QWidget>
#include <QPushButton>
#include <settings/CAlgorithmSettingsModel.h>

class CAlgorithmSettingsSaveLoadWidget : public QWidget {
    Q_OBJECT
private:
    int settingrow;
    CAlgorithmSettingsModel* settingmodel;
    QPushButton* save;
    QPushButton* load;
public:
  //TODO: Include CAlgorithmSettingsModel
  CAlgorithmSettingsSaveLoadWidget(QWidget* parent, int row, CAlgorithmSettingsModel& model);
  ~CAlgorithmSettingsSaveLoadWidget();
public slots:
  void savebutton();
  void loadbutton();

};

#endif // CALGORITHMSETTINGSSAVELOADWIDGET_H
