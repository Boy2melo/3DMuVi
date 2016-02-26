#include "CAlgorithmSettingsSaveLoadWidget.h"
#include <QPushButton>
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
    save = new QPushButton("save", this);
    load = new QPushButton("load", this);
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
    this->settingmodel->loadSettings(settingrow, QUrl());
}
void CAlgorithmSettingsSaveLoadWidget::savebutton()
{
    this->settingmodel->saveSettings(settingrow,QUrl());
}
