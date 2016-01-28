#include "CAlgoMessage.h"
#include "CLogController.h"
#include <ctime>

CAlgoMessage::CAlgoMessage(QString message, QString type = "INFO")
{
if( !((type == "ERROR") || (type == "WARNING") || (type == "DEBUG"))){
        type = "INFO"
}

QString time = get_timestamp();
CLogController::manageNewLogMessage(message, time, type);
delete this;
}
QString get_timestamp(){
    QString res = "";
    time_t rawtime;
    struct tm * timeinfo;
    char buffer [80];
    time (&rawtime);
    timeinfo = localtime (&rawtime);
    strftime (buffer,80,"%A, %B %d, %Y %I:%M:%S %p",timeinfo);
    res = buffer;
    return res;
}
