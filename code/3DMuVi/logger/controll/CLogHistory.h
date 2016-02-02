
#ifndef _CLOGHISTORY_H
#define _CLOGHISTORY_H
#include <vector>
#include <tuple>
#include <QString>
/**
 * @brief The CLogHistory class loghistory of all log messages with timestamp and type
 *  the history is used in the logger packet of the gui
 */
class CLogHistory {
public:  
    std::vector<std::tuple<QString,QString,QString>>& getHistory();
    

    void addHistory(QString message, QString time, QString type);
};

#endif //_CLOGHISTORY_H
