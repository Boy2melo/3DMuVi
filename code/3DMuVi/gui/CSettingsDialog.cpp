#include <QFileDialog>
#include <QMessageBox>

#include <logger/controll/CLogController.h>
#include <settings/CGlobalSettingController.h>

#include "CSettingsDialog.h"
#include "ui_CSettingsDialog.h"

CSettingsDialog::CSettingsDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::CSettingsDialog)
{
  CGlobalSettingController settings;
  unsigned int oldLogLevel = 0;
  bool oldLogLevelFound = false;
  unsigned int oldLogWindowEnabled = 0;
  bool oldLogWindowEnabledFound = false;
  unsigned int oldLogDataEnabled = 0;
  bool oldLogDataEnabledFound = false;

  ui->setupUi(this);

  ui->logLevelComboBox->addItem("Debug", QVariant(LOG_DEBUG));
  ui->logLevelComboBox->addItem("Info", QVariant(LOG_INFO));
  ui->logLevelComboBox->addItem("Warning", QVariant(LOG_WARN));
  ui->logLevelComboBox->addItem("Error", QVariant(LOG_ERROR));

  ui->resultDirectoryEdit->setText(settings.getSetting("resultDirectory"));

  oldLogLevel = settings.getSetting("minLogLevel").toUInt(&oldLogLevelFound);
  if(oldLogLevelFound)
  {
    int oldLogLevelIndex = ui->logLevelComboBox->findData(QVariant(oldLogLevel));

    ui->logLevelComboBox->setCurrentIndex(oldLogLevelIndex);
  }

  oldLogWindowEnabled = settings.getSetting("logWindowEnabled").toUInt(&oldLogWindowEnabledFound);
  if(oldLogWindowEnabledFound)
  {
    ui->logWindowCheckBox->setChecked(oldLogWindowEnabled);
  }

  oldLogDataEnabled = settings.getSetting("logDataEnabled").toUInt(&oldLogDataEnabledFound);
  if(oldLogDataEnabledFound)
  {
    ui->logDataCheckBox->setChecked(oldLogDataEnabled);
  }

  connect(ui->resultDirectoryButton, &QPushButton::clicked, this,
          &CSettingsDialog::onResultDirectoryButtonClicked);
}

CSettingsDialog::~CSettingsDialog()
{
  delete ui;
}

void CSettingsDialog::accept()
{
  updateResultDirectory();
  updateLogLevel();
  updateLogWindowEnabled();
  updateLogDataEnabled();

  QDialog::accept();
}

void CSettingsDialog::onResultDirectoryButtonClicked(bool)
{
  QUrl url = QFileDialog::getExistingDirectoryUrl(this, "Select result directory");

  ui->resultDirectoryEdit->setText(url.toString());
}

void CSettingsDialog::updateResultDirectory()
{
  CGlobalSettingController settings;
  QString resultDirectory = ui->resultDirectoryEdit->text();
  bool resultDirectorySuccess = settings.setSetting("resultDirectory", resultDirectory);

  if(!resultDirectorySuccess)
  {
    QMessageBox::warning(this, "Warning", "Unable to set result directory.");
  }
}

void CSettingsDialog::updateLogLevel()
{
  CGlobalSettingController settings;
  QString logLevel = ui->logLevelComboBox->currentData().toString();
  bool logLevelSuccess = settings.setSetting("minLogLevel", logLevel);

  CLogController::instance().setMinLoglevel(ui->logLevelComboBox->currentData().toUInt());

  if(!logLevelSuccess)
  {
    QMessageBox::warning(this, "Warning", "Unable to set log level.");
  }
}

void CSettingsDialog::updateLogWindowEnabled()
{
  CGlobalSettingController settings;
  QString logWindowEnabled;
  bool logWindowEnabledSuccess = false;

  switch(ui->logWindowCheckBox->checkState())
  {
    case Qt::Checked:
    case Qt::PartiallyChecked:
    {
      logWindowEnabled = QString(true);
      CLogController::instance().activateWindowlog();
    }
    break;

    case Qt::Unchecked:
    {
      logWindowEnabled = QString(false);
      CLogController::instance().deactivateWindowlog();
    }
  }
  logWindowEnabledSuccess = settings.setSetting("logWindowEnabled", logWindowEnabled);

  if(!logWindowEnabledSuccess)
  {
    QMessageBox::warning(this, "Warning", "Unable to set window log enabled state.");
  }
}

void CSettingsDialog::updateLogDataEnabled()
{
  CGlobalSettingController settings;
  QString logDataEnabled;
  bool logDataEnabledSuccess = false;

  switch(ui->logDataCheckBox->checkState())
  {
    case Qt::Checked:
    case Qt::PartiallyChecked:
    {
      logDataEnabled= QString(true);
      CLogController::instance().activateDatalog();
    }
    break;

    case Qt::Unchecked:
    {
      logDataEnabled = QString(false);
      CLogController::instance().deactivateDatalog();
    }
  }
  logDataEnabledSuccess = settings.setSetting("logDataEnabled", logDataEnabled);

  if(!logDataEnabledSuccess)
  {
    QMessageBox::warning(this, "Warning", "Unable to set data log enabled state.");
  }
}
