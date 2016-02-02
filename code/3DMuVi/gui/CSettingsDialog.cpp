#include <QFileDialog>
#include <QMessageBox>

#include <settings/CGlobalSettingController.h>

#include "CSettingsDialog.h"
#include "ui_CSettingsDialog.h"

CSettingsDialog::CSettingsDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::CSettingsDialog)
{
  ui->setupUi(this);
}

CSettingsDialog::~CSettingsDialog()
{
  delete ui;
}

void CSettingsDialog::accept()
{
  CGlobalSettingController settings;
  QString resultDirectory = ui->resultDirectoryEdit->text();
  bool resultDirectorySuccess = settings.setSetting("resultDirectory", resultDirectory);

  if(!resultDirectorySuccess)
  {
    QMessageBox::warning(this, "Warning", "Unable to set result directory.");
  }

  QDialog::accept();
}

void CSettingsDialog::onResultDirectoryButtonClicked(bool)
{
  QUrl url = QFileDialog::getExistingDirectoryUrl(this, "Select result directory");

  ui->resultDirectoryEdit->setText(url.toString());
}
