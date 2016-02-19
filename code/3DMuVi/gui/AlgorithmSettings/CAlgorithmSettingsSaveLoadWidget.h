#ifndef CALGORITHMSETTINGSSAVELOADWIDGET_H
#define CALGORITHMSETTINGSSAVELOADWIDGET_H

#include <QWidget>
#include <settings/CAlgorithmSettingsModel.h>

class CAlgorithmSettingsSaveLoadWidget : public QWidget {
private:
    int row;
    CAlgorithmSettingsModel model;
public:
  //TODO: Include CAlgorithmSettingsModel
  CAlgorithmSettingsSaveLoadWidget(int row, CAlgorithmSettingsModel& model);
public slots:
  void savebutton();
  void loadbutton();

};

#endif // CALGORITHMSETTINGSSAVELOADWIDGET_H
