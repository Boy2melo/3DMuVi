#include "CTestLoggerOutput.h"
#include <QDir>
#include <QStringList>

void CTestLoggerOutput::test()
{
    CLogController& controll = CLogController::instance();
    QDir workingDir = QDir(".");
    workingDir.mkdir("CTestResult");
    workingDir.cd("CTestResult");
    controll.manageNewLogMessage("A","B","C");
    controll.setLog(QUrl(workingDir.path()));
    bool logfound = false;
    QStringList allEntries = workingDir.entryList(QDir::AllEntries);
    for(QString entry : allEntries){
        if(entry == QString("log.txt")){
            logfound = true;
        }
    }

    QVERIFY2(logfound, "could not find log");
}
