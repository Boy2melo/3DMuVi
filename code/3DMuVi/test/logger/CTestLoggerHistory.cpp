#include "CTestLoggerHistory.h"


void CTestLoggerHistory::testgetHistory(){
 CLogHistory he;
 he.clearHistory();
 QString s0 = "Test message";
 he.addHistory("Test message","B","C");
 std::vector<std::tuple<QString,QString,QString>>  historylist =  he.getHistory();
 QCOMPARE(std::get<0> (historylist[0]),s0);
 he.clearHistory();
}

void CTestLoggerHistory::testaddHistory(){
    CLogHistory he;
    he.clearHistory();
    QString s1 = "Test message";
    QString s2 = "DEBUG";
    he.addHistory("Test message","B","C");
    he.addHistory("Test message","B","DEBUG");
    QCOMPARE(std::get<0> (he.getHistory()[0]),s1);
    QCOMPARE(std::get<2> (he.getHistory()[1]),s2);
    he.clearHistory();
}
