#include "CAlgoMessage.h"
#include "CLogController.h"
#include <ctime>

/**
 * @brief getTimestamp private function to create a timestamp
 * @return the timestamp as QString
 */
QString getTimestamp() {
    QString res = "";
    time_t rawtime;
    struct tm * timeinfo = nullptr;
    char buffer[80];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer, 80, "%A, %B %d, %Y %I:%M:%S %p", timeinfo);
    res = buffer;
    return res;
}
/**
 * @brief CAlgoMessage::CAlgoMessage constuctor of the class CAlgoMessage
 *        to create a new log message
 * @param message QString text to log
 * @param clcontroll CLogController the message is send to
 * @param type QString "ERROR", "WARNING", "DEBUG", "INFO" default value is "INFO"
 *        if type is not one of this 4 types it will be set to "INFO"
 */
CAlgoMessage::CAlgoMessage(QString message, CLogController& clcontroll, QString type) {
    if (!((type == "ERROR") || (type == "WARNING") || (type == "DEBUG"))) {
        type = "INFO";
    }

    QString time = "";
    time = getTimestamp();
    clcontroll.manageNewLogMessage(message, time, type);
    delete this;
}

