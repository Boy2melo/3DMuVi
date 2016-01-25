#include "CMainWindow.h"
#include "ui_CMainWindow.h"

CMainWindow::CMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::CMainWindow)
{
    ui->setupUi(this);
}

CMainWindow::~CMainWindow()
{
    delete ui;
}


//============================================================
/*!
@param checked
*/
//============================================================
void CMainWindow::onSaveWorkflow(bool checked)
{
}

//============================================================
/*!
@param checked
*/
//============================================================
void CMainWindow::onSaveWorkflowAs(bool checked)
{
}

//============================================================
/*!
@param checked
*/
//============================================================
void CMainWindow::onLoadImages(bool checked)
{
}

//============================================================
/*!
@param checked
*/
//============================================================
void CMainWindow::onAdvancedLoadFiles(bool checked)
{
}

//============================================================
/*!
@param checked
*/
//============================================================
void CMainWindow::onWorkflowSelected(bool checked)
{
}

//============================================================
/*!
@param checked
*/
//============================================================
void CMainWindow::onSettings(bool checked)
{
}

//============================================================
/*!
@param checked
*/
//============================================================
void CMainWindow::onAbout(bool checked)
{
}
