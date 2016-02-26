#ifndef CALGORITMSETTINGSVIEW_H
#define CALGORITMSETTINGSVIEW_H

#include <QTreeView>
#include <QPointer>
#include <workflow/workflow/aworkflow.h>
#include <settings/CAlgorithmSettingsModel.h>
#include <settings/CAlgorithmSettingController.h>

class CAlgorithmSettingsView : public QTreeView{
public: 
    /*!
   * \brief CAlgorithmSettingsView creates a new algorithmsettingsview
   * \param parent parent of the view
   */
  CAlgorithmSettingsView(QWidget* parent = nullptr);
  /*!
   * \brief setWorkflow sets the workflow and shows the view for the workflow
   * \param workflow the workflow
   */
  void setWorkflow(AWorkflow& workflow);
  /*!
   * \brief setAlgorithmController sets a Algorithmcontroller, must be done before setWorkflow();
   * \param controller the controller for the algorithms
   */
  void setAlgorithmController(CAlgorithmSettingController& controller);
    
public slots:
  void onAlgorithmChanged(int step);
 private:
  QPointer<CAlgorithmSettingsModel> model;
  QPointer<CAlgorithmSettingController> settingcontroller;
};

#endif // CALGORITMSETTINGSVIEW_H
