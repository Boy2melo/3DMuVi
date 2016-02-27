#ifndef CTESTCSETTINGSDIALOG_H
#define CTESTCSETTINGSDIALOG_H

#include <QTest>

/*!
\brief Test for CSettingsDialog.
\author Stefan Wolf

This class is a unit test for CSettingsDialog.
*/
class CTestCSettingsDialog : public QObject
{
  Q_OBJECT

private slots:
  /*!
  \brief Test for multiple functions.

  This is a test for multiple funcions of CSettingsDialog.
   */
  void test();

  /*!
  \brief Helper for the test.

  This is a helper for the test, which checks if the file selection dialog is shown and closes it.
  It doesn't has to be executed as a test.
   */
  void testResultDirectoryDialogShown();

  /*!
  \brief Helper for the about test.

  This is a helper for the test, which checks if an error message popped up and closes it.
  It doesn't has to be executed as a test.
   */
  void testCheckSettingsError();

private:
  bool mResultDirectoryDialogShown = false;
  const unsigned int N_SETTINGS = 4;
};

#endif // CTESTCSETTINGSDIALOG_H
