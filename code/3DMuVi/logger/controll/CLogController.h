
#ifndef _CLOGCONTROLLER_H
#define _CLOGCONTROLLER_H
#include "CLogHistory.h"
#include <QUrl>
#include <QObject>
#include <tuple>

#define __LOG(level) if(level >= CLogController::instance().getMinLogLevel()) CLogController::instance() << level <<

/**
 * @brief The CLogController class this class controll the logger,
 * manage the history and multiplexes the logmessages to the GUI (with singnal)
 * add it to the loghistory and calls qslog to log it in a text file if a destination is set
 */
class CLogController
    :public QObject {
    Q_OBJECT
private:
    CLogHistory mHistory;
    bool mDestSet;
    bool mLogToWindow;
    bool mLogToFile;
    uchar mCurrentLogLevel;
    uchar mMinLogLevel;
    void initilize();
    CLogController( const CLogController& ){}
    CLogController & operator = (const CLogController &){}
    CLogController(){
        initilize();
    }


public:
    static CLogController& instance() {
        static CLogController _instance;
        return _instance;
     }
    ~CLogController(){}

    void manageNewLogMessage(QString message, QString time, QString type);

    friend CLogController& operator<<(CLogController& logger, const QString&  message);
    friend CLogController& operator<<(CLogController& logger, const int& loglevel);

    CLogHistory& getHistory();

    void setLog(QUrl dest);
    void activateWindowlog();
    void deactivateWindowlog();
    void activateDatalog();
    void deactivateDatalog();
    void closeLog();

    void setMinLoglevel(uchar loglevel);
    uchar getMinLogLevel() const;

#define LOG_DEBUG 0
#define LOG_INFO 1
#define LOG_WARN 2
#define LOG_ERROR 3
    signals:
           void newLogMessage(QString message, QString time, QString type);


};

#endif //_CLOGCONTROLLER_H
