
#ifndef _CLOGHISTORY_H
#define _CLOGHISTORY_H

class CLogHistory {
public: 
    
    vector<tuple<QString,QString,Qstring>> getHistory();
    

    void addHistory(QString message, QString time, QString type);
};

#endif //_CLOGHISTORY_H
