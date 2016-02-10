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

  openMenu("Help", &CTestCMainWindow::aboutClickAbout);

  QVERIFY(mAboutSuccess);
}

void CTestCMainWindow::settings()
{
  CMainWindow mw;

  openMenu("Options", &CTestCMainWindow::settingsClickSettings);

  QVERIFY(mSettingsSuccess);
}

void CTestCMainWindow::workflowSelection()
{
  CMainWindow mw;
  std::vector<QGroupBox*> algorithmBoxes;

  openMenu("Workflow", &CTestCMainWindow::workflowSelectionClickFourPhase);

  for(QWidget* w : QApplication::allWidgets())
  {
    QGroupBox* groupBox = qobject_cast<QGroupBox*>(w);

    if(groupBox)
    {
      algorithmBoxes.push_back(groupBox);
    }
  }

  //needed to recalculate group boxes positions
  mw.show();
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

void CTestCMainWindow::aboutClickAbout()
{
  for(QWidget* w : QApplication::allWidgets())
  {
    QMenu* menu = qobject_cast<QMenu*>(w);

    if(menu && menu->title() == "Help")
    {
      QTimer::singleShot(WAITING_TIME, this, &CTestCMainWindow::aboutCheckMessageBox);
      QTest::mouseClick(menu, Qt::LeftButton, 0, QPoint(3, 3));
    }
  }
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

void CTestCMainWindow::settingsClickSettings()
{
  for(QWidget* w : QApplication::allWidgets())
  {
    QMenu* menu = qobject_cast<QMenu*>(w);

    if(menu && menu->title() == "Options")
    {
      QTimer::singleShot(WAITING_TIME, this, &CTestCMainWindow::settingsCheckDialog);
      QTest::mouseClick(menu, Qt::LeftButton, 0, QPoint(3, 3));
    }
  }
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

void CTestCMainWindow::workflowSelectionClickFourPhase()
{
  for(QWidget* w : QApplication::allWidgets())
  {
    QMenu* menu = qobject_cast<QMenu*>(w);

    if(menu && menu->title() == "Workflow")
    {
      QTest::mouseClick(menu, Qt::LeftButton, 0, QPoint(3, 3));
    }
  }
}

template<typename Func>
void CTestCMainWindow::openMenu(QString title, Func member)
{
  for(QWidget* w : QApplication::allWidgets())
  {
    QMenuBar* menuBar = qobject_cast<QMenuBar*>(w);

    if(menuBar)
    {
      QList<QMenu*> menus = menuBar->findChildren<QMenu*>(QString(), Qt::FindDirectChildrenOnly);

      for(QMenu* m : menus)
      {
        if(m->title() == title)
        {
          QTimer::singleShot(WAITING_TIME, this, member);
          m->exec();
        }
      }
    }
  }
}

void CTestCMainWindow::workflowSelectionCheckGroupBox(QGroupBox* groupBox, QString reference)
{
  QVERIFY(groupBox);

  if(groupBox)
  {
    QCOMPARE(groupBox->title(), reference);
  }
}
