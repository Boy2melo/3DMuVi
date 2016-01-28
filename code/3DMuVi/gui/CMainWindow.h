#ifndef CMAINWINDOW_H
#define CMAINWINDOW_H

#include <memory>

#include <QMainWindow>

#include <workflow/workflow/aworkflow.h>

namespace Ui {
  class CMainWindow;
}

class CMainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit CMainWindow(QWidget *parent = 0);
  ~CMainWindow();

private:
  Ui::CMainWindow *ui;
  std::unique_ptr<AWorkflow> mWorkflow;
  std::unique_ptr<AContextDataStore> mDataStore;

private slots:
  void onSaveWorkflow();
  void onSaveWorkflowAs();
  void onLoadImages();
  void onAdvancedLoadFiles();
  void onWorkflowSelected();
  void onSettings();
  void onAbout();
};

#endif // CMAINWINDOW_H
