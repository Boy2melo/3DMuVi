#ifndef CALGORITMSETTINGSVIEW_H
#define CALGORITMSETTINGSVIEW_H

#include <QTreeView>

#include <workflow/workflow/aworkflow.h>

class CAlgoritmSettingsView : public QTreeView {
public: 
  void CAlgorithmSettingsView(AWorkflow& workflow);
  void setWorkflow(AWorkflow& workflow);
    
public slots:
  void onAlgorithmChanged(int step);
};

#endif // CALGORITMSETTINGSVIEW_H
