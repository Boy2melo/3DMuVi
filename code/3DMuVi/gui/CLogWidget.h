#ifndef CLOGWIDGET_H
#define CLOGWIDGET_H

#include <QPlainTextEdit>

class CLogWidget : public QPlainTextEdit
{
public slots:
    void onNewLogMessage(QString message, QString time, QString type);
    void onStateChangedDebug(int state);
    void onStateChangedInfo(int state);
    void onStateChangedWarning(int state);
    void onStateChangedError(int state);
};

#endif // CLOGWIDGET_H
