#ifndef CTESTCMAINWINDOW_H
#define CTESTCMAINWINDOW_H

#include <QTest>
#include <QGroupBox>

/*!
\brief Test for CMainWindow.
\author Stefan Wolf

This class is a unit test for CMainWindow.
*/
class CTestCMainWindow : public QObject
{
  Q_OBJECT

private slots:
  /*!
  \brief Test for about function.

  This is a test for the about message.
   */
  void about();

  /*!
  \brief Test for settings function.

  This is a test for the settings dialog.
   */
  void settings();

  /*!
  \brief Test for workflow selection function.

  This is a test for the workflow selection function. It selects a workflow and checks if
  appropriate group boxes have been created. This test assumes that CFourPhaseWorkflow is
  the first worklfow in the menu.
   */
  void workflowSelection();

protected slots:
  /*!
  \brief Helper for the about test.

  This is a helper for the about test, which clicks on the about entry in the helpt menu.
   */
  void aboutClickAbout();

  /*!
  \brief Helper for the about test.

  This is a helper for the about test, which checks if the message is shown and closes it.
   */
  void aboutCheckMessageBox();

  /*!
  \brief Helper for the settings test.

  This is a helper for the settings test, which clicks on the settings entry in the options menu.
   */
  void settingsClickSettings();

  /*!
  \brief Helper for the settings test.

  This is a helper for the settings test, which checks if the dialog is shown and closes it.
   */
  void settingsCheckDialog();

  /*!
  \brief Helper for the workflow selection test.

  This is a helper for the workflow selection test, which clicks on the four phase entry in the
  workflow menu.
   */
  void workflowSelectionClickFourPhase();

private:
  template<typename Func>
  void openMenu(QString title, Func member);
  void workflowSelectionCheckGroupBox(QGroupBox* groupBox, QString reference);

  bool mAboutSuccess = false;
  bool mSettingsSuccess = false;
};

#endif // CTESTCMAINWINDOW_H
