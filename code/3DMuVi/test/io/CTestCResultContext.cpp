#include "CTestCResultContext.h"

void CTestCResultContext::initTestCase()
{
    workingDir = QDir(".");
    workingDir.mkdir("CTestResultContextTempData");
    workingDir.cd("CTestResultContextTempData");

    algoController = new CAlgorithmSettingController(QUrl("."));
    algoController->setSetting(QString("algorithmConfig"),QJsonObject());
    globalController = new CGlobalSettingController();

    CResultContext resultContext(QUrl(workingDir.absolutePath()),algoController,globalController);

    std::shared_ptr<FeatureMatch> fM (new FeatureMatch);
    fM->push_back(std::make_tuple((uint64_t)1,1.0,1.0,(uint64_t)1));
    std::shared_ptr<CDataFeature> dataFeaturePacket (new CDataFeature);
    dataFeaturePacket->setFeatureMatch(fM);
    resultContext.addDataPacket(dataFeaturePacket);
}

void CTestCResultContext::test()
{
    bool globalConfigFound = false;
    bool algorithmConfigFound = false;
    bool logFound = false;
    bool dataTypeFolderFound = false;

    QStringList resultContextDirs = workingDir.entryList(QDir::AllDirs);
    //three entries: ., .. and the real directory
    QVERIFY2(resultContextDirs.size() == 3,"Maybe there is no or too much resultcontext directories.");

    workingDir.cd(resultContextDirs.at(2));
    QStringList allEntries = workingDir.entryList(QDir::AllEntries);
    workingDir.cdUp();

    for(QString entry : allEntries){
        if(entry == QString("globalconfig.json")){
            globalConfigFound = true;
        }
        if(entry == QString("algorithmConfig.json")){
            algorithmConfigFound = true;
        }
        if(entry == DT_FEATURE_MATCH){
            dataTypeFolderFound = true;
        }
        if(entry == QString("log.txt")){
            logFound = true;
        }
    }

    QVERIFY2(globalConfigFound,"Could not find global config.");
    QVERIFY2(algorithmConfigFound,"Could not find algorithm config.");
    QVERIFY2(dataTypeFolderFound,"Could not find serialized data folder.");
    QVERIFY2(logFound,"Could not find log file.");
}

void CTestCResultContext::cleanupTestCase(){
    removeDir(workingDir.absolutePath());
    delete(algoController);
    delete(globalController);
}

bool CTestCResultContext::removeDir(const QString & dirName)
{
    bool result = true;
    QDir dir(dirName);

    if (dir.exists(dirName)) {
        Q_FOREACH(QFileInfo info, dir.entryInfoList(QDir::NoDotAndDotDot | QDir::System | QDir::Hidden  | QDir::AllDirs | QDir::Files, QDir::DirsFirst)) {
            if (info.isDir()) {
                result = removeDir(info.absoluteFilePath());
            }
            else {
                result = QFile::remove(info.absoluteFilePath());
            }

            if (!result) {
                return result;
            }
        }
        result = dir.rmdir(dirName);
    }
    return result;
}
