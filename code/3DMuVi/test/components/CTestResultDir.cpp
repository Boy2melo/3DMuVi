#include "CTestResultDir.h"
#include <io/CResultContext.h>
#include<workflow/workflow/datapackets/CDataDepth.h>
#include<workflow/workflow/datapackets/CDataFeature.h>
#include<workflow/workflow/idatapacket.h>
using FeatureMatch = std::vector<std::tuple<uint64_t, float, float, uint32_t>>;
using DepthMaps = std::vector<std::tuple<uint32_t, QImage>>;


void CTestResultDir::test(){

QUrl path("results");
CAlgorithmSettingController asctr(path);
CAlgorithmSettingController* algoSCT = &asctr;
CGlobalSettingController gbs;
CGlobalSettingController* globalSCT = &gbs;
CResultContext rConT = CResultContext(path,algoSCT,globalSCT);

std::shared_ptr<DepthMaps> depthM(new DepthMaps);
uint32_t x = 1;
QImage y = QImage(200,200,QImage::Format_RGB32);
y.fill(Qt::darkYellow);
std::tuple<uint32_t, QImage> dME (x,y);
depthM.get()->push_back(dME);


FeatureMatch match;
uint64_t a = 327178;
uint32_t b = 321;
std::tuple<uint64_t, float, float, uint32_t> matchE (a,3.1,2.4,b);
match.push_back(matchE);

CDataFeature dfp = CDataFeature();
dfp.setFeatureMatch(std::make_shared<FeatureMatch>(match));
rConT.addDataPacket(std::make_shared<CDataFeature>(dfp));

CDataDepth ddp = CDataDepth();
ddp.setDepthMap(depthM);
rConT.addDataPacket(std::make_shared<CDataDepth>(ddp));

QDir dirx;
QCOMPARE(dirx.cd(path.path()),true);
dirx.cd(dirx.entryList(QDir::AllDirs).last());
QCOMPARE(dirx.exists(dfp.getDataType()),true);
QCOMPARE(dirx.exists(ddp.getDataType()),true);


}
