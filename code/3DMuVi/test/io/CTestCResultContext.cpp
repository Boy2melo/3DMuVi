#include "CTestCResultContext.h"

void CTestCResultContext::initTestCase()
{
    CLogController& lc = CLogController::instance();
    CAlgorithmSettingController* algoController = new CAlgorithmSettingController(QUrl("."));
    CGlobalSettingController* globalController = new CGlobalSettingController();
    CResultContext asd(QUrl("."),algoController,globalController);
}

void CTestCResultContext::test()
{
    //noch zu implementieren.
    QCOMPARE(true, false);
}

void CTestCResultContext::cleanupTestCase(){
    QDir toCleanFolder = QDir::current();
    QStringList toDeleteFolders = toCleanFolder.entryList(QDir::AllDirs);
    for(QString toDeleteFolder : toDeleteFolders){
        toCleanFolder.rmdir(toDeleteFolder);
    }
}
