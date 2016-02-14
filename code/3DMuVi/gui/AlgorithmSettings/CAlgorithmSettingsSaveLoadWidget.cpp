#include "CAlgorithmSettingsSaveLoadWidget.h"
#include "ui_CAlgorithmSettingsSaveLoad.h"

//============================================================
/*!
@param row row of the algorithm
@param model the Algorthmsettingsmodel
 */
//============================================================
void CAlgorithmSettingsSaveLoadWidget::CAlgorithmSettingsSaveLoadWidget(int row, CAlgorithmSettingsModel& model),
    ui(new Ui::CAlgortihmSettingsSaveLoad)

{
    ui->setupUi(this);
    this->model = model;
    this->row = row;

    //signalslotsconnecten
}

void loadbutton()
{
    this.model.laodSetting(row, QUrl());
}
void savebutton()
{
    this.model.saveSetting(row,QUrl());
}
