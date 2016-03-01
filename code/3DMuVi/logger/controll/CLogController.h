
#ifndef _CLOGCONTROLLER_H
#define _CLOGCONTROLLER_H
#include "CLogHistory.h"
#include <QUrl>
#include <QObject>
#include <tuple>

#define __LOG(level) if(level >= CLogController::instance().getMinLogLevel()) CLogController::instance() << level <<

 /*!
 * \brief The CLogController class this class controll the logger,
 * \author Tim Brodbeck
 * LogController is Singleton
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
    // copy Constructor is not possible because of  :public QObject
    // CLogController( const CLogController& ){}
    // CLogController & operator = (const CLogController &){}
    CLogController(){
        initilize();
    }


public:
    /*!
     * \brief instance
     * \return CLogController&
     * CLogController is Singleton can be called by CLogController::instance()
     */
    static CLogController& instance() {
        static CLogController _instance;
        return _instance;
     }
    /*!
     *\brief destructor
     */
    ~CLogController(){}
    /*!
     * \brief manageNewLogMessage
     * \param message
     * \param time
     * \param type
     * write into history and in data if the Windowlog and Datalog is enabled
     * only writes in data if url is set before by setLog
     */
    void manageNewLogMessage(QString message, QString time, QString type);
    /*!
     * \brief operator <<
     * \param logger
     * \param message
     * \return
     * << operator for easy message writing
     */
    friend CLogController& operator<<(CLogController& logger, const QString&  message);
    /*!
     * \brief operator <<
     * \param logger
     * \param loglevel
     * \return
     * << operator for easy loglevel change
     */
    friend CLogController& operator<<(CLogController& logger, const int& loglevel);
    /*!
     * \brief getHistory
     * \return CLogHistory&
     * returns the History Object of the messages
     */
    CLogHistory& getHistory();

    /*!
     * \brief setLog
     * \param dest
     * set Url for Datalog
     */
    void setLog(QUrl dest);
    /*!
     * \brief activateWindowlog
     * activate Historylog
     */
    void activateWindowlog();
    /*!
     * \brief deactivateWindowlog
     * deactivate Historylog
     */
    void deactivateWindowlog();
    /*!
     * \brief activateDatalog
     * activate log into Data
     */
    void activateDatalog();
    /*!
     * \brief deactivateDatalog
     * deactivate log into Data
     */
    void deactivateDatalog();
    /*!
     * \brief closeLog
     * close Log directory
     */
    void closeLog();
    /*!
     * \brief setMinLoglevel
     * \param loglevel
     * set minLogLevelS
     */
    void setMinLoglevel(uchar loglevel);
    /*!
     * \brief getMinLogLevel
     * \return uchar minLogLevel
     */
    uchar getMinLogLevel() const;

#define LOG_DEBUG 0
#define LOG_INFO 1
#define LOG_WARN 2
#define LOG_ERROR 3
    signals:
             /*!
            * \brief newLogMessage
            * \param message
            * \param time
            * \param type
            * Signal to sending UI new Log Messages
            */
           void newLogMessage(QString message, QString time, QString type);


};

#endif //_CLOGCONTROLLER_H
