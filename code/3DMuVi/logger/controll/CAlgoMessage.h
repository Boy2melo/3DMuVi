#ifndef CALGOMESSAGE_H
#define CALGOMESSAGE_H
#include <QString>
#include "CLogController.h"
/**
 * @brief The CAlgoMessage class
 * this class is used to create log messages.
 * to create a new log message, create a new object of this class
 * you call the constructor with message, controllerobject, messagetype
 * it creates the timestamp and calls the controller
 * after the controller is called, this object deletes itself !
 */
class CAlgoMessage
{
public:
    CAlgoMessage(QString message,CLogController& clcontroll, QString type = "INFO");
private:
    QString get_timestamp();
};

#endif // CALGOMESSAGE_H
