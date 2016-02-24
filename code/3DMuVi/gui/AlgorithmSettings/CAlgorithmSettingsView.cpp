#include "CAlgorithmSettingsView.h"
#include <settings/CAlgorithmSettingsModel.h>

//============================================================
/*!
@param workflow
 */
//============================================================
CAlgorithmSettingsView::CAlgorithmSettingsView(QWidget* parent) : QTreeView(parent)
{

}

//============================================================
/*!
@param workflow
 */
//============================================================
void CAlgorithmSettingsView::setWorkflow(AWorkflow& workflow)
{

    //CAlgorithmSettingsModel model;
    //setModel(model);

}

//============================================================
/*!
@param step
 */
//============================================================
void CAlgorithmSettingsView::onAlgorithmChanged(int step)
{
    QUrl url;

    //this->model.loadSettings(step, url);
}
