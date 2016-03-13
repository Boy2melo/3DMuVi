#include <gui/CLogWidget.h>

/*!
\brief Test for CLogWidget.
\author Stefan Wolf

This class is a unit test for CLogWidget.
*/
class CTestCLogWidget : public QObject
{
  Q_OBJECT

private slots:
  void initTestCase();
  void init();

  /*!
  \brief Test for onNewLogMessage.

  This is a test for onNewLogMessage with valid input data.
   */
  void onNewLogMessage();

  /*!
  \brief Test for onStateChangedDebug.

  This is a test for onStateChangedDebug with valid input data.
   */
  void onStateChangedDebug();

  /*!
  \brief Test for onStateChangedInfo.

  This is a test for onStateChangedInfo with valid input data.
   */
  void onStateChangedInfo();

  /*!
  \brief Test for onStateChangedWarning.

  This is a test for onStateChangedWarning with valid input data.
   */
  void onStateChangedWarning();

  /*!
  \brief Test for onStateChangedError.

  This is a test for onStateChangedError with valid input data.
   */
  void onStateChangedError();

  void cleanup();
  void cleanupTestCase();

private:
  CLogWidget* mWidget;
};
