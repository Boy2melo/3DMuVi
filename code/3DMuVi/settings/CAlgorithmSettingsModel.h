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
    void saveSettings(int row, QUrl filename) override;
    void saveSettingsEx(int row, QUrl filename);
    void loadSettings(int row, QUrl filename) override;
    /*!
     * \brief algorithmChanged changes the model, if an algorithm is changed
     * \param step number of the algorithm
     */
    void algorithmChanged(int step);
    /*!
     * \brief validateAll Plugins
     * \return true if all correct
     */
    bool validateAll();
    bool setData(const QModelIndex &index, const QVariant &value, int role) override;

    /*!
     * \brief insertName inserts the name of the algorithm into the model
     * \param row place of the algorithm
     */
    void insertName(int row);
};
#endif //CALGORITMSETTINGSMODEL_H
