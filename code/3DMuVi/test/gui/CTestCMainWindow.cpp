#include <iostream>

#include <QMessageBox>
#include <QMenuBar>
#include <QTimer>
#include <QThread>
#include <QGroupBox>

#include <gui/CMainWindow.h>
#include <gui/CSettingsDialog.h>

#include "CTestCMainWindow.h"

#define WAITING_TIME 10
#define WAITING_TIME_LONG 20

void CTestCMainWindow::about()
{
  CMainWindow mw;

  for(QWidget* w : QApplication::allWidgets())
  {
    QMenuBar* menuBar = qobject_cast<QMenuBar*>(w);

    if(menuBar)
    {
      QTest::mouseClick(menuBar, Qt::LeftButton, 0, QPoint(200, 3));
    }
  }

  for(QWidget* w : QApplication::allWidgets())
  {
    QMenu* menu = qobject_cast<QMenu*>(w);

    if(menu)
    {
      QTimer::singleShot(WAITING_TIME, this, &CTestCMainWindow::aboutCheckMessageBox);
      QTest::mouseClick(menu, Qt::LeftButton, 0, QPoint(3, 3));
    }
  }

  QVERIFY(mAboutSuccess);
}

void CTestCMainWindow::aboutCheckMessageBox()
{
  for(QWidget* w : QApplication::topLevelWidgets())
  {
    QMessageBox* messageBox = qobject_cast<QMessageBox*>(w);

    if(messageBox)
    {
      if(messageBox->windowTitle() == "About 3DMuVi")
      {
        mAboutSuccess = true;
      }

      QTest::keyClick(messageBox, Qt::Key_Enter);
    }
  }
}

void CTestCMainWindow::settings()
{
  CMainWindow mw;

  for(QWidget* w : QApplication::allWidgets())
  {
    QMenuBar* menuBar = qobject_cast<QMenuBar*>(w);

    if(menuBar)
    {
      QTest::mouseClick(menuBar, Qt::LeftButton, 0, QPoint(150, 3));
    }
  }

  for(QWidget* w : QApplication::allWidgets())
  {
    QMenu* menu = qobject_cast<QMenu*>(w);

    if(menu)
    {
      QTimer::singleShot(WAITING_TIME, this, &CTestCMainWindow::settingsCheckDialog);
      QTest::mouseClick(menu, Qt::LeftButton, 0, QPoint(3, 3));
    }
  }

  QVERIFY(mSettingsSuccess);
}

void CTestCMainWindow::settingsCheckDialog()
{
  for(QWidget* w : QApplication::topLevelWidgets())
  {
    CSettingsDialog* dialog = qobject_cast<CSettingsDialog*>(w);

    if(dialog)
    {
      if(dialog->windowTitle() == "Settings")
      {
        mSettingsSuccess = true;
      }

      QTest::keyClick(dialog, Qt::Key_Escape);
    }
  }
}

void CTestCMainWindow::workflowSelection()
{
  CMainWindow mw;
  std::vector<QGroupBox*> algorithmBoxes;

  mw.show();

  for(QWidget* w : QApplication::allWidgets())
  {
    QMenuBar* menuBar = qobject_cast<QMenuBar*>(w);

    if(menuBar)
    {
      QTest::mouseClick(menuBar, Qt::LeftButton, 0, QPoint(100, 3));
    }
  }

  for(QWidget* w : QApplication::allWidgets())
  {
    QMenu* menu = qobject_cast<QMenu*>(w);

    if(menu)
    {
      QTest::mouseClick(menu, Qt::LeftButton, 0, QPoint(3, 3));
    }
  }

  for(QWidget* w : QApplication::allWidgets())
  {
    QGroupBox* groupBox = qobject_cast<QGroupBox*>(w);

    if(groupBox)
    {
      algorithmBoxes.push_back(groupBox);
    }
  }

  //needed to recalculate group boxes positions
  QApplication::processEvents();

  std::sort(algorithmBoxes.begin(), algorithmBoxes.end(),
            [](QGroupBox* a, QGroupBox* b) -> bool
  {
    return a->y() < b->y();
  });

  workflowSelectionCheckGroupBox(algorithmBoxes.at(0), "Feature Matcher");
  workflowSelectionCheckGroupBox(algorithmBoxes.at(1), "Depth Mapper");
  workflowSelectionCheckGroupBox(algorithmBoxes.at(2), "Pose Estimator");
  workflowSelectionCheckGroupBox(algorithmBoxes.at(3), "PCL Reconstructor");
}

void CTestCMainWindow::workflowSelectionCheckGroupBox(QGroupBox* groupBox, QString reference)
{
  QVERIFY(groupBox);

  if(groupBox)
  {
    QCOMPARE(groupBox->title(), reference);
  }
}
