#include "CTestLoggerHistory.h"




void CTestLoggerHistory::testgetHistory(){
 CLogHistory hector;
 CLogHistory& he = hector;
 QString s0 = "Test message";
 he.addHistory("Test message","B","C");
 std::vector<std::tuple<QString,QString,QString>>  historylist =  he.getHistory();
 QCOMPARE(std::get<0> (historylist[0]),s0);
}

void CTestLoggerHistory::testaddHistory(){
    CLogHistory hector2;
    CLogHistory& he2 = hector2;
    QString s1 = "Test message";
    QString s2 = "DEBUG";
    he2.addHistory("Test message","B","C");
    he2.addHistory("Test message","B","DEBUG");
    std::vector<std::tuple<QString,QString,QString>>  historylist2 =  he2.getHistory();
    QCOMPARE(std::get<0> (historylist2[0]),s1);
    QCOMPARE(std::get<2> (historylist2[1]),s2);
}
