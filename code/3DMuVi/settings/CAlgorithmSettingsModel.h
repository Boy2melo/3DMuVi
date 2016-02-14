#ifndef CALGORITMSETTINGSMODEL_H
#define CCALGORITMSETTINGSMODEL_H

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
    //AWorkflow workflow;
public:
    /*!
     * \brief CAlgorithmSettingsModel construct a settingsmodel
     * \param manager
     */
    CAlgorithmSettingsModel(AWorkflow& workflow, CAlgorithmSettingController& controller);
    CAlgorithmSettingsModel(QObject *parent, QVector<QJsonObject> list);
    void saveSettings(int row, QUrl filename);
    void loadSettings(int row, QUrl filename);

};
#endif //CALGORITMSETTINGSMODEL_H
