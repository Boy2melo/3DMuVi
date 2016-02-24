
#include "CLogHistory.h"

using namespace std;


/**
 * @brief CLogHistory::getHistory returns a reference of the log history
 * @return vector<tuple<QString,QString,QString>>& each tuple<message,time,type> in vector
 * is for 1 complete log message
 */
vector<tuple<QString,QString,QString>>& CLogHistory::getHistory() {
            vector<tuple<QString,QString,QString>>& x = currenthistory;
    return x;
}

/**
 * @brief CLogHistory::addHistory add a message to the History at the end
 * @param message text to log
 * @param time  timestamp
 * @param type "INFO","ERROR","WARNING","DEBUG"
 */
void CLogHistory::addHistory(QString message, QString time, QString type) {
    tuple<QString,QString,QString> t (message,time,type);
    currenthistory.push_back(t);
}
