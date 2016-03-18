#include "CAlgorithmSettingsSaveLoadWidget.h"
#include <QPushButton>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QFile>
//============================================================
/*!
@param row row of the algorithm
@param model the Algorthmsettingsmodel
 */
//============================================================
CAlgorithmSettingsSaveLoadWidget::CAlgorithmSettingsSaveLoadWidget(QWidget* parent, int row, CAlgorithmSettingsModel& model):
    QWidget(parent)
{
    settingmodel = &model;
    settingrow = row;
    setLayout(new QHBoxLayout);
    save = new QPushButton("save", this);
    load = new QPushButton("load", this);
    layout()->addWidget(save);
    layout()->addWidget(load);
    connect(load, &QPushButton::clicked,
            this, &CAlgorithmSettingsSaveLoadWidget::loadbutton);
    connect(save, &QPushButton::clicked,
            this, &CAlgorithmSettingsSaveLoadWidget::savebutton);
}
CAlgorithmSettingsSaveLoadWidget::~CAlgorithmSettingsSaveLoadWidget()
{
    delete save;
    delete load;
}
void CAlgorithmSettingsSaveLoadWidget::loadbutton()
{
    QUrl url = QFileDialog::getOpenFileUrl(this, "Select Parameterfile");
    this->settingmodel->loadSettings(settingrow, url);
  ;
}
void CAlgorithmSettingsSaveLoadWidget::savebutton()
{
    QUrl url = QFileDialog::getSaveFileName(this, tr("Save File"), "",
                                             tr("QJason File(*.json);;All Files (*)"));
    this->settingmodel->saveSettings(settingrow,url);
}
