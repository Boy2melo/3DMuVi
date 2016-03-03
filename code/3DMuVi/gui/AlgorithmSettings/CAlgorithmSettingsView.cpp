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
    int algorithms = workflow.getStepCount();
    for (int i = 0; i < algorithms; i++) {
        QJsonObject object = workflow.getStep(i)->GetParameterJson();
        model->loadQJson(object);
        setIndexWidget(model->index(i, 1), new CAlgorithmSettingsSaveLoadWidget(this, i, *model));
    }
    this->show();
}

CAlgorithmSettingController* CAlgorithmSettingsView::getAlgorithmController()
{
    return settingcontroller.data();
}

//============================================================
/*!
@param step
 */
//============================================================
void CAlgorithmSettingsView::onAlgorithmChanged(int step)
{
    this->hide();
    model->algorithmChanged(step);
    QModelIndex index = this->rootIndex().child(step, 2);
    this->setIndexWidget(index, new CAlgorithmSettingsSaveLoadWidget(this, step, *model));
    this->show();
}
