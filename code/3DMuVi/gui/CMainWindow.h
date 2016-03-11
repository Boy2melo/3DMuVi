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

  /*!
  \brief setLoadImage

  Will load images from a input dataset and set them to the imagePreviewWidget and
  algorithmSelector.
  \param url to the input dataset.
  */
  void setLoadImage(QUrl url);

private:
  Ui::CMainWindow *ui;
  std::unique_ptr<AWorkflow> mWorkflow;
  CContextDataStore* mDataStore = nullptr;

  void setWorkflow(AWorkflow* workflow);

private slots:
  void onLoadImages();
  void onWorkflowSelected();
  void onSettings();
  void onAbout();

  void onDataStoreFinished(CContextDataStore* dataStorage);
};

#endif // CMAINWINDOW_H
