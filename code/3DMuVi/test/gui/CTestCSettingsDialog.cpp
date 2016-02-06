#include <iostream>

#include <QFileDialog>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTimer>

#include <gui/CSettingsDialog.h>
#include <settings/CGlobalSettingController.h>

#include "CTestCSettingsDialog.h"

#define WAITING_TIME 10
#define WAITING_TIME_LONG 20

void CTestCSettingsDialog::test()
{
  CSettingsDialog settingsDialog;
  CGlobalSettingController settingController;

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

    if(resultDirectoryEdit)
    {
      QTest::keyClicks(resultDirectoryEdit, "/tmp/");
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

  QCOMPARE(settingController.getSetting("resultDirectory"), QString("/tmp/"));
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
      QTest::keyClick(messageBox, Qt::Key_Enter);
    }
  }
}
