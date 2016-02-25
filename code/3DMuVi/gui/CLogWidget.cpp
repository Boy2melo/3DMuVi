#include <logger/controll/CLogHistory.h>

#include "CLogWidget.h"


CLogWidget::CLogWidget(QWidget* parent) : QPlainTextEdit(parent)
{
    CLogController& mpLoggerref  = CLogController::instance();
    mpLogger = &mpLoggerref;

    setReadOnly(true);
    connect(mpLogger,  &CLogController::newLogMessage, this, &CLogWidget::onNewLogMessage);
}

CLogWidget::~CLogWidget()
{
  // delete mpLogger;
}

void CLogWidget::onNewLogMessage(QString message, QString time, QString type)
{
  if(type == "DEBUG" && mShowDebug)
  {
    appendPlainText(time + ": " + type + ": " + message);
  }
  else if(type == "INFO" && mShowInfo)
  {
    appendPlainText(time + ": " + type + ": " + message);
  }
  else if(type == "WARNING" && mShowWarning)
  {
    appendPlainText(time + ": " + type + ": " + message);
  }
  else if(type == "ERROR" && mShowError)
  {
    appendPlainText(time + ": " + type + ": " + message);
  }
}

void CLogWidget::onStateChangedDebug(int state)
{
  updateViewState(state, &mShowDebug);
}

void CLogWidget::onStateChangedInfo(int state)
{
  updateViewState(state, &mShowInfo);
}

void CLogWidget::onStateChangedWarning(int state)
{
  updateViewState(state, &mShowWarning);
}

void CLogWidget::onStateChangedError(int state)
{
  updateViewState(state, &mShowError);
}

void CLogWidget::updateViewState(int state, bool* internalState)
{
  if(state == Qt::Unchecked)
  {
    *internalState = false;
  }
  else if(state == Qt::Checked || state == Qt::PartiallyChecked)
  {
    *internalState = true;
  }

  updateLog();
}

void CLogWidget::updateLog()
{
  //CLogHistory history;
  std::vector<std::tuple<QString, QString, QString>> pastMessages = mpLogger->getHistory().getHistory();

  clear();
  for(std::tuple<QString, QString, QString> m : pastMessages)
  {
    onNewLogMessage(std::get<0>(m), std::get<1>(m), std::get<2>(m));
  }
}
