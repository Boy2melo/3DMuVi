#ifndef CALGORITMSETTINGSVIEW_H
#define CALGORITMSETTINGSVIEW_H

#include <QTreeView>
#include <QPointer>
#include <QTemporaryDir>
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
  /*! \brief destructor
   */
  ~CAlgorithmSettingsView();
  /*!
   * \brief setWorkflow sets the workflow and shows the view for the workflow
   * \param workflow the workflow
   */
  void setWorkflow(AWorkflow& workflow);
  /*!
   * \brief gets the algorithmconrtoller of the view
   * \return the conrtoller
   */
  CAlgorithmSettingController* getAlgorithmController();
    
public slots:
  void onAlgorithmChanged(int step);
 private:
  QPointer<CAlgorithmSettingsModel> model;
  QPointer<CAlgorithmSettingController> settingcontroller;
  QTemporaryDir temp;
};

#endif // CALGORITMSETTINGSVIEW_H
