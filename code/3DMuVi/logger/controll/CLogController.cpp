/**
 * Project Untitled
 */


#include "CLogController.h"
#include "CLogHistory.h"
#include "QSLog.h"
#include "QSLogDest.h"
#include <QDir>

CLogHistory history;
QSLogging::Logger& logger;
bool destSet;
/**
 * CLogController implementation
 */
CLogController::CLogController(){

    destSet = false;
}

//add to History, GUI-Signal and add to Qslog
void CLogController::manageNewLogMessage(QString message, QString time, QString type) {
    h::addHistory(message,time,type);
    newLogMessage(message, time, type);
    if( ! destSet ){
        return;
    }else{
    if(type == "INFO"){
    QSLogging::QLOG_INFO() << message;
    }
    if(type == "ERROR"){
    QSLogging::QLOG_ERROR() << message;
    }
    if(type == "WARNING"){
    QSLogging::QLOG_WARN() << message;
    }
    if(type == "DEBUG"){
    QSLogging::QLOG_DEBUG() << message;
    }
    }
}
void CLogController::setHistory(CLogHistory h){
    history = h;
}

/**
 * @param dest
 */
void CLogController::setLog(QUrl dest) {
    logger =  QSLogging::Logger::instance();
    logger.setLoggingLevel(QSLogging::TraceLevel);
    const QString sLogPath(QDir(dest));
    QSLogging::DestinationPtr fileDestination( QSLogging::DestinationFactory::MakeFileDestination(
         sLogPath, EnableLogRotation, MaxSizeBytes(512), MaxOldLogCount(2)));
    QSLogging::DestinationPtr debugDestination( QSLogging::DestinationFactory::MakeDebugOutputDestination());
    QSLogging::DestinationPtr functorDestination( QSLogging::DestinationFactory::MakeFunctorDestination(&logFunction));
    logger.addDestination(debugDestination);
    logger.addDestination(fileDestination);
    logger.addDestination(functorDestination);

    destSet = true;

}

void CLogController::closeLog() {
    destSet = false;
    QSLogging::Logger::destroyInstance();

}

/**
 * @param message
 * @param time
 * @param type
 * @return signal
 */
void CLogController::newLogMessage(QString message, QString time, QString type) {

}
