#include <iostream>

#include <QCheckBox>
#include <QComboBox>
#include <QFileDialog>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTimer>

#include <gui/CSettingsDialog.h>
#include <settings/CGlobalSettingController.h>
#include <logger/controll/CLogController.h>

#include "CTestCSettingsDialog.h"

#define WAITING_TIME 10
#define WAITING_TIME_LONG 20

void CTestCSettingsDialog::test()
{
  CGlobalSettingController settings;
  settings.setSetting("resultDirectory", "");
  settings.setSetting("minLogLevel", QString::number(LOG_INFO));
  settings.setSetting("logWindowEnabled", QString::number(0));
  settings.setSetting("logDataEnabled", QString::number(0));

  CSettingsDialog settingsDialog;

  for(QWidget* w : QApplication::allWidgets())
  {
    QPushButton* directoryButton = qobject_cast<QPushButton*>(w);

    if(directoryButton && directoryButton->text() == "...")
    {
      QTimer::singleShot(WAITING_TIME, this, &CTestCSettingsDialog::testResultDirectoryDialogShown);
      QTest::mouseClick(directoryButton, Qt::LeftButton);
    }
  }

  QVERIFY(mResultDirectoryDialogShown);

  for(QWidget* w : QApplication::allWidgets())
  {
    QLineEdit* resultDirectoryEdit = qobject_cast<QLineEdit*>(w);
    QComboBox* logLevelComboBox = qobject_cast<QComboBox*>(w);
    QCheckBox* logEnabledCheckBox = qobject_cast<QCheckBox*>(w);

    if(resultDirectoryEdit)
    {
      resultDirectoryEdit->setFocus(Qt::OtherFocusReason);
      QTest::keyClicks(resultDirectoryEdit, "/tmp/");
    }

    if(logLevelComboBox)
    {
      logLevelComboBox->setCurrentText("Error");
    }

    if(logEnabledCheckBox && (logEnabledCheckBox->text() == "Window logging" ||
                              logEnabledCheckBox->text() == "Data logging"))
    {
      logEnabledCheckBox->setChecked(true);
    }
  }

  for(QWidget* w : QApplication::allWidgets())
  {
    QPushButton* acceptButton = qobject_cast<QPushButton*>(w);

    if(acceptButton && acceptButton->text() == "&OK")
    {
      QTimer::singleShot(WAITING_TIME_LONG, this, &CTestCSettingsDialog::testCheckSettingsError);
      QTest::mouseClick(acceptButton, Qt::LeftButton);
    }
  }

  CGlobalSettingController settingController;
  QCOMPARE(settingController.getSetting("resultDirectory"), QString("/tmp/"));
  QCOMPARE(settingController.getSetting("minLogLevel"), QString::number(LOG_ERROR));
  QCOMPARE(settingController.getSetting("logWindowEnabled"), QString::number(1));
  QCOMPARE(settingController.getSetting("logDataEnabled"), QString::number(1));
  QCOMPARE(CLogController::instance().getMinLogLevel(), static_cast<uchar>(LOG_ERROR));
}

void CTestCSettingsDialog::testResultDirectoryDialogShown()
{
  for(QWidget* w : QApplication::topLevelWidgets())
  {
    QFileDialog* dialog = qobject_cast<QFileDialog*>(w);

    if(dialog && dialog->windowTitle() == "Select result directory")
    {
      mResultDirectoryDialogShown = true;

      QTest::keyClick(dialog, Qt::Key_Escape);
    }
  }
}

void CTestCSettingsDialog::testCheckSettingsError()
{
  for(QWidget* w : QApplication::topLevelWidgets())
  {
    QMessageBox* messageBox = qobject_cast<QMessageBox*>(w);

    if(messageBox && messageBox->windowTitle() == "Warning")
    {
      QTimer::singleShot(WAITING_TIME_LONG, this, &CTestCSettingsDialog::testCheckSettingsError);
      QTest::keyClick(messageBox, Qt::Key_Enter);
    }
  }
}
