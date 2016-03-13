#ifndef CLOGWIDGET_H
#define CLOGWIDGET_H

#include <QPlainTextEdit>

#include <logger/controll/CLogController.h>

/*!
\brief A widget which shows the log of a program.
\author Stefan Wolf

This widget show the log of a program in a text box.
*/
class CLogWidget : public QPlainTextEdit
{
  Q_OBJECT

public:
  //============================================================
  /*!
  \brief Initializes the widget.

  This constructor initzializes the widget and connects this widget with the logger.
  @param parent The widget which should be the parent of this one.
  */
  //============================================================
  explicit CLogWidget(QWidget* parent = nullptr);

  //============================================================
  /*!
  \brief Cleans the widget.

  Destroys all creted objects.
  */
  //============================================================
  virtual ~CLogWidget();

public slots:
  //============================================================
  /*!
  \brief Adds a new message to the log widget.

  Appends a new message at the end of the log widget.
  @param message The message to add.
  @param time The time of the messages creation.
  @param type The messages type.
  */
  //============================================================
  void onNewLogMessage(QString message, QString time, QString type);

  //============================================================
  /*!
  \brief Changes the view state of debug messages.

  If state is Qt::Checked, all debug messages are shown. If state is Qt::Unchecked, no debug
  messages are shown.
  @param state Should be either Qt::Checked or Qt::Unchecked.
  */
  //============================================================
  void onStateChangedDebug(int state);

  //============================================================
  /*!
  \brief Changes the view state of info messages.

  If state is Qt::Checked, all info messages are shown. If state is Qt::Unchecked, no info
  messages are shown.
  @param state Should be either Qt::Checked or Qt::Unchecked.
  */
  //============================================================
  void onStateChangedInfo(int state);

  //============================================================
  /*!
  \brief Changes the view state of debug messages.

  If state is Qt::Checked, all warning messages are shown. If state is Qt::Unchecked, no warning
  messages are shown.
  @param state Should be either Qt::Checked or Qt::Unchecked.
  */
  //============================================================
  void onStateChangedWarning(int state);

  //============================================================
  /*!
  \brief Changes the view state of debug messages.

  If state is Qt::Checked, all error messages are shown. If state is Qt::Unchecked, no error
  messages are shown.
  @param state Should be either Qt::Checked or Qt::Unchecked.
  */
  //============================================================
  void onStateChangedError(int state);

private:
  void updateViewState(int state, bool* internalState);
  void updateLog();

  CLogController* mpLogger = nullptr;
  bool mShowDebug = true, mShowInfo = true, mShowWarning = true, mShowError = true;
};

#endif // CLOGWIDGET_H
