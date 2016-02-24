
#ifndef _CLOGCONTROLLER_H
#define _CLOGCONTROLLER_H
#include "CLogHistory.h"
#include <QUrl>
#include <QObject>
#include <tuple>
/**
 * @brief The CLogController class this class controll the logger,
 * manage the history and multiplexes the logmessages to the GUI (with singnal)
 * add it to the loghistory and calls qslog to log it in a text file if a destination is set
 */
class CLogController
         :public QObject
{
  Q_OBJECT
public: 
    CLogController();

    void manageNewLogMessage(QString message, QString time, QString type);

    friend CLogController& operator<<(CLogController& logger,const QString&  message );
    friend CLogController& operator<<(CLogController& logger,const int& loglevel );

    CLogHistory& getHistory();
    
    void setLog(QUrl dest);
    void activateWindowlog();
    void deactivateWindowlog();
    void activateDatalog();
    void deactivateDatalog();
    void closeLog();
signals:
    void newLogMessage(QString message, QString time, QString type);


};

#endif //_CLOGCONTROLLER_H
