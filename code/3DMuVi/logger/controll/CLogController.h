
#ifndef _CLOGCONTROLLER_H
#define _CLOGCONTROLLER_H

class CLogController {
public: 
    CLogController();

    void manageNewLogMessage(QString message, QString time, QString type);
    
    void setHistory(CLogHistory h);
    
    void setLog(QUrl dest);
    
    void closeLog();
signals:
    void newLogMessage(QString message, QString time, QString type);
};

#endif //_CLOGCONTROLLER_H
