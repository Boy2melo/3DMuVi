#include "CLogController.h"
#include "CLogHistory.h"
#include "../qslogging/QsLog.h"
#include "../qslogging/QsLogDest.h"
#include <QDir>
#include <iostream>
#include <ctime>
using namespace QsLogging;


/**
 * @brief CLogController::CLogController constructor
 * sets the private attrubute mDestSet to false;
 * use the function setLog(QUrl dest) to set an dest for the logger file
 */
void CLogController::initilize() {
    mHistory = CLogHistory();
    mDestSet = false;
    mCurrentLogLevel = LOG_INFO;
    mLogToWindow = true;
    mLogToFile = true;
    mMinLogLevel = LOG_INFO;
}

/*
CLogController::CLogController() {
    mHistory = CLogHistory();
    mDestSet = false;
    mCurrentLogLevel = LOG_INFO;
    mLogToWindow = true;
    mLogToFile = true;
    mMinLogLevel = LOG_INFO;
}
*/
void CLogController::activateWindowlog() {
    mLogToWindow = true;
}
void CLogController::deactivateWindowlog() {
    mLogToWindow = false;
}
void CLogController::activateDatalog() {
    mLogToFile = true;
}
void CLogController::deactivateDatalog() {
    mLogToFile = false;
}

CLogController& operator<< (CLogController& logger, const int& loglev) {
    logger.mCurrentLogLevel = static_cast<uchar>(loglev);
    return logger;
}

CLogController& operator<< (CLogController& logger, const QString& message) {
    if (logger.mCurrentLogLevel < logger.mMinLogLevel) {
        return logger;
    }

    QString type;
    switch (logger.mCurrentLogLevel) {
    case LOG_DEBUG:
        type = "DEBUG";
        break;
    case LOG_INFO:
        type = "INFO";
        break;
    case LOG_WARN:
        type = "WARNING";
        break;
    case LOG_ERROR:
        type = "ERROR";
        break;
    default:
        type = "INFO";
        break;
    }
    logger.mCurrentLogLevel = LOG_INFO;

    QString mtime = "";
    time_t rawtime;
    struct tm *timeinfo = nullptr;
    char buffer[80];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer, 80, "%A, %B %d, %Y %I:%M:%S %p", timeinfo);
    mtime = buffer;

    logger.manageNewLogMessage(message, mtime, type);
    return logger;
}


/**
 * @brief CLogController::manageNewLogMessage this function manage new Logmessages
 *        from CAlgoMessage objects. It adds this objects to the History, calls QsLogging
 *        to write it to a file, if Destination is set, and send a Signal to the GUI
 *        with the new message
 *
 * @param message QString text to log
 * @param time    QString timestamp of the logmessage
 * @param type    QString "INFO", "ERROR" , "WARNING" or "DEBUG"
 */
void CLogController::manageNewLogMessage(QString message, QString time, QString type) {

    if (!((type == "ERROR") || (type == "WARNING") || (type == "DEBUG"))) {
        type = "INFO";
    }

    if (mLogToWindow) {
        mHistory.addHistory(message, time, type);
        emit(newLogMessage(message, time, type));
    }

    if (mDestSet && mLogToFile) {
        if (type == "INFO") {
            QLOG_INFO() << message;
        }
        if (type == "ERROR") {
            QLOG_ERROR() << message;
        }
        if (type == "WARNING") {
            QLOG_WARN() << message;
        }
        if (type == "DEBUG") {
            QLOG_DEBUG() << message;
        }
    }
}
/**
 * @brief CLogController::getHistory give the History where the log messages are saved temp
 * @return CLogHistory& reference to the mHistory object;
 */
CLogHistory& CLogController::getHistory() {
    CLogHistory& refhistory = mHistory;
    return refhistory;
}

/**
 * @brief CLogController::setLog this function is used
 *        to create a output destination for the txt files
 *        it activates the file logging
 * @param dest QUrl with the destination
 */
void CLogController::setLog(QUrl dest) {

    Logger& logger = Logger::instance();
    logger.setLoggingLevel(TraceLevel);
    const QString sLogPath = dest.toString();
    DestinationPtr fileDestination(DestinationFactory::MakeFileDestination(
        sLogPath, EnableLogRotation, MaxSizeBytes(20000000), MaxOldLogCount(2000000)));
    DestinationPtr debugDestination(DestinationFactory::MakeDebugOutputDestination());
    //DestinationPtr functorDestination(DestinationFactory::MakeFunctorDestination(&logFunction));
    logger.addDestination(debugDestination);
    logger.addDestination(fileDestination);
    // logger.addDestination(functorDestination);

    mDestSet = true;

}
/**
 * @brief CLogController::closeLog this is used to close the log Destination
 * and stop the File Logging
 */
void CLogController::closeLog() {
    mDestSet = false;
    Logger::destroyInstance();
}

void CLogController::setMinLoglevel(uchar loglevel) {
    mMinLogLevel = loglevel;
}

uchar CLogController::getMinLogLevel() const {
    return mMinLogLevel;
}
