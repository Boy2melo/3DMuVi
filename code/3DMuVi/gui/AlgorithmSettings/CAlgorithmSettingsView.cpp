#include "CAlgorithmSettingsView.h"
#include "CAlgorithmSettingsSaveLoadWidget.h"
#include <QModelIndex>
#include <QUrl>
//============================================================
/*!
@param workflow
 */
 //============================================================
CAlgorithmSettingsView::CAlgorithmSettingsView(QWidget* parent) : QTreeView(parent) {
    QTemporaryDir temporary_dir;
    if (temporary_dir.isValid()) {
        QUrl url = QUrl(temporary_dir.path());
        settingcontroller = QPointer<CAlgorithmSettingController>(new CAlgorithmSettingController((url)));
    }
    stepcount = 0;
}

CAlgorithmSettingsView::~CAlgorithmSettingsView() {
    temp.remove();
}

//============================================================
/*!
@param workflow
 */
 //============================================================
void CAlgorithmSettingsView::setWorkflow(AWorkflow& workflow) {
    this->reset();
    model = QPointer<CAlgorithmSettingsModel>(new CAlgorithmSettingsModel(workflow, *settingcontroller));
    this->setModel(model);
    stepcount = workflow.getStepCount();

    for (int i = 0; i < stepcount; i++) {
        IPlugin* plugin = workflow.getStep(i);
        QJsonObject object = plugin ? plugin->GetParameterJson() : QJsonObject();
        model->loadQJson(object);
        model->insertName(i);
        if (plugin) {
        plugin->getAlgorithm()->setParameters(object);
        }
        setIndexWidget(model->index(i, 1), new CAlgorithmSettingsSaveLoadWidget(this, i, *model));
    }
    this->show();
}

CAlgorithmSettingController* CAlgorithmSettingsView::getAlgorithmController() {
    return settingcontroller.data();
}

//============================================================
/*!
@param step
 */
 //============================================================
void CAlgorithmSettingsView::onAlgorithmChanged(int step) {
    this->hide();
    model->algorithmChanged(step);
    this->reset();
    this->setModel(model);
    for (int i = 0; i < stepcount; i++)
    {
        this->setIndexWidget(model->index(i, 1),
                             new CAlgorithmSettingsSaveLoadWidget(this, step, *model));
    }
    this->show();
}
