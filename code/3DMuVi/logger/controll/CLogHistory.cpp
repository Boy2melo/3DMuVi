/**
 * Project Untitled
 */


#include "CLogHistory.h"

/**
 * CLogHistory implementation
 */
vector<tuple<QString,QString,Qstring>> history;

/**
 * @return vector<tuple<QString,QString,Qstring>>
 */
vector<tuple<QString,QString,Qstring>> CLogHistory::getHistory() {

    return history;
}

/**
 * @param message
 * @param time
 * @param type
 */
void CLogHistory::addHistory(QString message, QString time, QString type) {

    history.push_back(tuple<message,time,type>);
}
