#include "CAlgorithmSettingsView.h"

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
    model = QPointer<CAlgorithmSettingsModel>(new CAlgorithmSettingsModel(workflow, *settingcontroller));
    this->setModel(model);
    //saveloadwidget einbinden
    this->show();
}

void CAlgorithmSettingsView::setAlgorithmController(CAlgorithmSettingController& controller)
{
    settingcontroller = QPointer<CAlgorithmSettingController>(&controller);
}

//============================================================
/*!
@param step
 */
//============================================================
void CAlgorithmSettingsView::onAlgorithmChanged(int step)
{
    model->algorithmChanged(step);
    this->show();
}
