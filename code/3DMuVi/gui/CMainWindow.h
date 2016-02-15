#ifndef CMAINWINDOW_H
#define CMAINWINDOW_H

#include <memory>

#include <QMainWindow>

#include <workflow/workflow/aworkflow.h>

namespace Ui {
  class CMainWindow;
}

/*!
\brief 3D-MuVis MainWindow.
\author Stefan Wolf

This class is the programs MainWindow and contains all Widgets.
*/
class CMainWindow : public QMainWindow
{
  Q_OBJECT

public:
  /*!
  \brief CMainWindows constructor.

  Initializes the window.
  */
  explicit CMainWindow(QWidget *parent = 0);

  /*!
  \brief CMainWindows destructor.

  Cleans up all members.
  */
  ~CMainWindow();

private:
  Ui::CMainWindow *ui;
  std::unique_ptr<AWorkflow> mWorkflow;
  std::unique_ptr<CContextDataStore> mDataStore;

private slots:
  void onSaveWorkflow();
  void onSaveWorkflowAs();
  void onLoadImages();
  void onAdvancedLoadFiles();
  void onWorkflowSelected();
  void onSettings();
  void onAbout();

  void onDataStoreFinished(CContextDataStore* dataStorage);
};

#endif // CMAINWINDOW_H
