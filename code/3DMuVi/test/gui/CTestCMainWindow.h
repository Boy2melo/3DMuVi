#ifndef CTESTCMAINWINDOW_H
#define CTESTCMAINWINDOW_H

#include <QTest>

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
  \brief Helper for the about test.

  This is a helper for the about test, which checks if the message is shown and closes it.
  It doesn't has to be executed as a test.
   */
  void aboutCheckMessageBox();

  /*!
  \brief Test for settings function.

  This is a test for the settings dialog.
   */
  void settings();

  /*!
  \brief Helper for the settings test.

  This is a helper for the settings test, which checks if the dialog is shown and closes it.
  It doesn't has to be executed as a test.
   */
  void settingsCheckDialog();

private:
  bool mAboutSuccess = false;
  bool mSettingsSuccess = false;
};

#endif // CTESTCMAINWINDOW_H
