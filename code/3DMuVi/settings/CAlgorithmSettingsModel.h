#ifndef CALGORITMSETTINGSMODEL_H
#define CCALGORITMSETTINGSMODEL_H

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>
#include <QString>

#include <workflow/workflow/aworkflow.h>
/*!
 * \brief The CAlgorithmSettingsModel class
 */
class CAlgorithmSettingsModel : public CQJsonModel
{
private:
    AWorkflow workflow;
public:
    /*!
     * \brief CAlgorithmSettingsModel construct a settingsmodel with the given workflow
     * \param workflow  workflow for the model
     */
    CAlgorithmSettingsModel(AWorkflow& workflow);

};
#endif //CALGORITMSETTINGSMODEL_H
