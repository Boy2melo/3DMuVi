#include "CTestLoggerControll.h"
CLogController c;
CLogController& controll = c;
CLogHistory& h = controll.getHistory();
void CTestLoggerControll::initTestCase()
{
    QUrl x;
    controll.setLog(x);
}
void CTestLoggerControll::testgetHistory(){
    std::vector<std::tuple<QString,QString,QString>>  historylist =  h.getHistory();
    std::vector<std::tuple<QString,QString,QString>>  historylist2 =  controll.getHistory().getHistory();

    QCOMPARE(historylist.size(),historylist2.size());
}
void CTestLoggerControll::testmanageNewLogMessage(){
    controll.manageNewLogMessage("A","B","");
    controll.manageNewLogMessage("A","B","C");
    controll.manageNewLogMessage("A","B","DEBUG");

    CLogHistory h2;
    h2.addHistory("A","B","INFO");
    h2.addHistory("A","B","INFO");
    h2.addHistory("A","B","DEBUG");

    std::vector<std::tuple<QString,QString,QString>>  historylist =  h.getHistory();
    std::vector<std::tuple<QString,QString,QString>>  historylist2 =  h2.getHistory();

    QCOMPARE(historylist.size(),historylist2.size());

        for (int j = 0; j < 3; j++){
    QCOMPARE(std::get<0> (historylist[j]),std::get<0> (historylist2[j]));
    QCOMPARE(std::get<1> (historylist[j]),std::get<1> (historylist2[j]));
    QCOMPARE(std::get<1> (historylist[j]),std::get<1> (historylist2[j]));
   }
}
