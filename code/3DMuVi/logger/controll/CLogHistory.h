
#ifndef _CLOGHISTORY_H
#define _CLOGHISTORY_H
#include <vector>
#include <tuple>
#include <QString>
/*!
 * \brief The CLogHistory class loghistory of all log messages with timestamp and type
 *  the history is used in the logger packet of the gui
 * \author Tim Brodbeck
 */
class CLogHistory {

    std::vector<std::tuple<QString, QString, QString>> currenthistory;
public:  

    /*!
     * \brief CLogHistory::getHistory returns a reference of the log history
     * \return vector<tuple<QString,QString,QString>>& each tuple<message,time,type> in vector
     * is for 1 complete log message
     */
    std::vector<std::tuple<QString,QString,QString>>& getHistory();
    /*!
     * \brief CLogHistory::addHistory add a message to the History at the end
     * \param message text to log
     * \param time  timestamp
     * \param type "INFO","ERROR","WARNING","DEBUG"
     */
    void addHistory(QString message, QString time, QString type);
    /*!
     * \brief clearHistory
     * clear History List needed for QTUnitests
     */
    void clearHistory();
};

#endif //_CLOGHISTORY_H
