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
    /*!
   * \brief CAlgorithmSettingsSaveLoadWidget creates a new widget
   * \param parent parent of the widget
   * \param row row in the model
   * \param model the settingsmodel, in which the widget is used
   */
  CAlgorithmSettingsSaveLoadWidget(QWidget* parent, int row, CAlgorithmSettingsModel& model);
  ~CAlgorithmSettingsSaveLoadWidget();
public slots:
  void savebutton();
  void loadbutton();

};

#endif // CALGORITHMSETTINGSSAVELOADWIDGET_H
