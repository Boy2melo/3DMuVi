
#include "CLogHistory.h"

using namespace std;


vector<tuple<QString,QString,QString>>& CLogHistory::getHistory() {
            vector<tuple<QString,QString,QString>>& x = currenthistory;
    return x;
}


void CLogHistory::addHistory(QString message, QString time, QString type) {
    tuple<QString,QString,QString> t (message,time,type);
    currenthistory.push_back(t);
}
void CLogHistory::clearHistory(){
    currenthistory.clear();
}
