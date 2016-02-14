#include <QMessageBox>
#include <QFileDialog>

#include <io/CInputDataSet.h>
#include <workflow/workflow/cworkflowmanager.h>

#include "CSettingsDialog.h"

#include "CMainWindow.h"
#include "ui_CMainWindow.h"

//============================================================
/*!
\brief CMainWindows constructor.

Initializes the window.
*/
//============================================================
CMainWindow::CMainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::CMainWindow)
{
  QVector<QString> workflows;

  ui->setupUi(this);

  workflows = CWorkflowManager::Instance()->getAvailableWorkflows();
  for (QString w : workflows)
  {
    ui->menuWorkflow->addAction(w, this, SLOT(onWorkflowSelected()));
  }

  connect(ui->actionLoadImages, &QAction::triggered, this, &CMainWindow::onLoadImages);
//TODO: advanced load files
  connect(ui->actionSettings, &QAction::triggered, this, &CMainWindow::onSettings);
  connect(ui->actionAbout, &QAction::triggered, this, &CMainWindow::onAbout);
}

//============================================================
/*!
\brief CMainWindows destructor.

Cleans up all members.
*/
//============================================================
CMainWindow::~CMainWindow()
{
    delete ui;
}


void CMainWindow::onSaveWorkflow()
{
  //TODO: Doesn't need to be implemented
}

void CMainWindow::onSaveWorkflowAs()
{
  //TODO: Doesn't need to be implemented
}

void CMainWindow::onLoadImages()
{
  QUrl url = QFileDialog::getExistingDirectoryUrl(this, "Select image directory");

  if(!url.isEmpty() && mWorkflow)
  {
    CInputDataSet* dataSet = new CInputDataSet(url);
    std::vector<std::tuple<uint32_t, QImage, CImagePreviewItem>>* images =
      dataSet->getInputImages();
    std::vector<CImagePreviewItem*> imageItems;

    //CContextDataStore* dataStore = mWorkflow->addDataStore(dataSet);

    for(std::tuple<uint32_t, QImage, CImagePreviewItem> i : *images)
    {
      imageItems.push_back(&(std::get<2>(i)));
    }

    ui->imagePreviewWidget->setImages(imageItems);
  }
}

void CMainWindow::onAdvancedLoadFiles()
{
  //TODO: Doesn't need to be implemented
}

void CMainWindow::onWorkflowSelected()
{
  QAction* callingAction = qobject_cast<QAction*>(sender());

  if(callingAction)
  {
    AWorkflow* workflow = CWorkflowManager::Instance()->getWorkflow(callingAction->text());

    ui->algorithmSelector->setWorkflow(*workflow);
    mWorkflow.reset(workflow);
  }
}

void CMainWindow::onSettings()
{
  CSettingsDialog dialog(this);
  dialog.exec();
}

void CMainWindow::onAbout()
{
  QMessageBox::about(this, "About 3DMuVi", "3DMuVi is a framework for testing "\
                     "3D-reconstruction-algorithms.\nCopyright 2016 Tim Brodbeck, Jens Manig, "\
                     "Grigori Schapoval, Nathanael Schneider, Laurenz Thiel and Stefan Wolf.");
}
