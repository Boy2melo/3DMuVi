#ifndef CALGORITMSETTINGSMODEL_H
#define CALGORITMSETTINGSMODEL_H

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>
#include <QString>

#include "CQJsonModel.h"
#include <workflow/workflow/aworkflow.h>
#include <settings/CAlgorithmSettingController.h>
/*!
 * \brief The CAlgorithmSettingsModel class
 */
class CAlgorithmSettingsModel : public CQJsonModel
{
private:
    AWorkflow* workflow;
    CAlgorithmSettingController* settingcontroller;
public:
    /*!
     * \brief CAlgorithmSettingsModel construct a settingsmodel
     * \param workflow the  workflow for the model
     * \param controller the algorithmsettingcontroller
     */
    CAlgorithmSettingsModel(AWorkflow& workflow, CAlgorithmSettingController& controller);
    CAlgorithmSettingsModel(QObject *parent, QVector<QJsonObject> list);
    void saveSettings(int row, QUrl filename);
    void loadSettings(int row, QUrl filename);
    /*!
     * \brief algorithmChanged changes the model, if an algorithm is changed
     * \param step number of the algorithm
     */
    void algorithmChanged(int step);

};
#endif //CALGORITMSETTINGSMODEL_H
