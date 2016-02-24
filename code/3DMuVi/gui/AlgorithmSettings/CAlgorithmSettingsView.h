#ifndef CALGORITMSETTINGSVIEW_H
#define CALGORITMSETTINGSVIEW_H

#include <QTreeView>

#include <workflow/workflow/aworkflow.h>
#include <settings/CAlgorithmSettingsModel.h>

class CAlgorithmSettingsView : public QTreeView {
public: 
  CAlgorithmSettingsView(QWidget* parent = nullptr);
  void setWorkflow(AWorkflow& workflow);
    
public slots:
  void onAlgorithmChanged(int step);
 private:
  //CAlgorithmSettingsModel model;
};

#endif // CALGORITMSETTINGSVIEW_H
