#ifndef CALGORITMSETTINGSVIEW_H
#define CALGORITMSETTINGSVIEW_H

#include <QTreeView>
#include <QPointer>
#include <workflow/workflow/aworkflow.h>
#include <settings/CAlgorithmSettingsModel.h>
#include <settings/CAlgorithmSettingController.h>

class CAlgorithmSettingsView : public QTreeView{
public: 
  CAlgorithmSettingsView(QWidget* parent = nullptr);
  void setWorkflow(AWorkflow& workflow);
  void setAlgorithmController(CAlgorithmSettingController& controller);
    
public slots:
  void onAlgorithmChanged(int step);
 private:
  QPointer<CAlgorithmSettingsModel> model;
  QPointer<CAlgorithmSettingController> settingcontroller;
};

#endif // CALGORITMSETTINGSVIEW_H
