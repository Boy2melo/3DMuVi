#include "CAlgorithmSettingsView.h"
#include "CAlgorithmSettingsSaveLoadWidget.h"
#include <QModelIndex>
#include <QUrl>
//============================================================
/*!
@param workflow
 */
//============================================================
CAlgorithmSettingsView::CAlgorithmSettingsView(QWidget* parent) : QTreeView(parent)
{
    QTemporaryDir temp;
    if(temp.isValid()) {
        QUrl url = QUrl(temp.path());
    settingcontroller = QPointer<CAlgorithmSettingController>(new CAlgorithmSettingController((url)));
    }
}
CAlgorithmSettingsView::~CAlgorithmSettingsView()
{
    temp.remove();
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
    //saveloadwidget einbinden setIndexWidget(Modelindex, widget)
    int algorithms = workflow.getStepCount();
    int i = 0;
    QModelIndex index = this->rootIndex().child(0, 2);
    while (i <= algorithms) {
        if (index.isValid()){
            this->setIndexWidget(index, new CAlgorithmSettingsSaveLoadWidget(0, i, *model));
        }
        i++;
        index = index.sibling(i, 2);
    }
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
