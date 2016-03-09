#ifndef CTESTALGORITHMEXECUTION_H
#define CTESTALGORITHMEXECUTION_H


#include <QTest>
#include <QStatusBar>
#include <QPushButton>
#include "gui/CStepComboBox.h"

/*!
\brief Testing algorithm execution with dummyplugins which each wait 2 seconds.
\author Laurenz Thiel

Info: This test expect the following plugins in the testfolder. (featureMatch_t34, pose_t34, depthMap_t34, dummyFusion_t34)
The source code for the plugins: https://github.com/boitumeloruf/3DMuVi/tree/master/code/3DMuVi%20Plugins/dummy_t34
This test is specified as user interface test T3.4 in the specification sheet.
*/
class CTestAlgorithmExecution : public QObject
{
  Q_OBJECT

private slots:
  /*!
  \brief Runs the test.
  */
  void test();

private:
  static bool setPluginIfValid(QComboBox* comboBox, QString pluginName);
};

#endif // CTESTALGORITHMEXECUTION_H
