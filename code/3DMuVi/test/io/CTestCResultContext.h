#include <QTest>
#include <io/CResultContext.h>
#include <workflow/workflow/datapackets/CDataFeature.h>
#include "workflow/workflow/ccontextdatastore.h"

using FeatureMatch = std::vector<std::tuple<uint64_t, float, float, uint32_t>>;

/*!
\brief Test for CResultContext.
\author Laurenz Thiel

This class is a unit test for CResultContext.
*/
class CTestCResultContext : public QObject
{
  Q_OBJECT

private slots:
  void initTestCase();
  void cleanupTestCase();

  /*!
  \brief Test for blank.

  Blank description.
   */
  void test();

private:
  bool removeDir(const QString & dirName);
  CAlgorithmSettingController* algoController;
  CGlobalSettingController* globalController;
  QDir workingDir;
};
