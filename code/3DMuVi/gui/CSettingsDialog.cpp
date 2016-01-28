#include "CSettingsDialog.h"
#include "ui_CSettingsDialog.h"

//============================================================
/*!
@param parent
*/
//============================================================
CSettingsDialog::CSettingsDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::CSettingsDialog)
{
  ui->setupUi(this);
}

//============================================================
/*!
*/
//============================================================
CSettingsDialog::~CSettingsDialog()
{
  delete ui;
}


//============================================================
/*!
*/
//============================================================
void CSettingsDialog::accept()
{

}

//============================================================
/*!
@param checked
*/
//============================================================
void CSettingsDialog::onResultDirectoryButtonClicked(bool checked)
{

}
