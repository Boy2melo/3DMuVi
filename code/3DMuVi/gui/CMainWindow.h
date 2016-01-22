#ifndef CMAINWINDOW_H
#define CMAINWINDOW_H

#include <QMainWindow>

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

private slots:
  void onSaveWorkflow(bool checked);
  void onSaveWorkflowAs(bool checked);
  void onLoadImages(bool checked);
  void onAdvancedLoadFiles(bool checked);
  void onWorkflowSelected(bool checked);
  void onSettings(bool checked);
  void onAbout(bool checked);
};

#endif // CMAINWINDOW_H
