
#ifndef _CLOGCONTROLLER_H
#define _CLOGCONTROLLER_H

class CLogController {
public: 
    
    void refreshLog();
    
    void setLog(QUrl dest);
    
    void closeLog();
    
    signal newLogMessage(QString message, QString time, QString type);
};

#endif //_CLOGCONTROLLER_H
