#include "CTestLoggerControll.h"

void CTestLoggerControll::initTestCase()
{
    CLogController::instance().getHistory().clearHistory();
}
void CTestLoggerControll::testgetHistory(){
    //clear History (Singleton in QT Unitests..)
    CLogController::instance().getHistory().clearHistory();
    CLogHistory h = CLogController::instance().getHistory();
    h.addHistory("X","Y","");
    std::vector<std::tuple<QString,QString,QString>>  historylist =  h.getHistory();

    QCOMPARE(historylist.empty(),false);
    //clear History (Singleton in QT Unitests..)
    h.clearHistory();
}
void CTestLoggerControll::testmanageNewLogMessage(){
    //clear History (Singleton in QT Unitests..)
    CLogController::instance().getHistory().clearHistory();
    CLogHistory h = CLogController::instance().getHistory();

    CLogController::instance().manageNewLogMessage("Hallo","00:34","");
    CLogController::instance().manageNewLogMessage("TestNachricht","06:20","C");
    CLogController::instance().manageNewLogMessage("Fatal Error","20:34","DEBUG");

    QString text[3] = {"Hallo","TestNachricht","Fatal Error"};
    QString time[3] = {"00:34","06:20","20:34"};
    QString type[3] = {"INFO","INFO","DEBUG"};

    std::vector<std::tuple<QString,QString,QString>>  historylist =  h.getHistory();

        for (int j = 0; j < 3; j++){
    QCOMPARE(std::get<0> (historylist[j]),text[j]);
    QCOMPARE(std::get<1> (historylist[j]),time[j]);
    QCOMPARE(std::get<2> (historylist[j]),type[j]);
    //clear History (Singleton in QT Unitests..)
     CLogController::instance().getHistory().clearHistory();
   }
}
