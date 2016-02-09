#ifndef CALGORITMSETTINGSMODEL_H
#define CCALGORITMSETTINGSMODEL_H

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>
#include <QString>

#include "CQJsonModel.h"
#include <workflow/plugin/cpluginmanager.h>
/*!
 * \brief The CAlgorithmSettingsModel class
 */
class CAlgorithmSettingsModel : public CQJsonModel
{
private:
    //CPluginManager manager;
public:
    /*!
     * \brief CAlgorithmSettingsModel construct a settingsmodel
     * \param manager
     */
    CAlgorithmSettingsModel(CPluginManager& manager);
    CAlgorithmSettingsModel(QObject *parent, QVector<QJsonObject> list);

};
#endif //CALGORITMSETTINGSMODEL_H
