#ifndef CALGORITMSETTINGSMODEL_H
#define CCALGORITMSETTINGSMODEL_H

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>
#include <QString>

#include <workflow/workflow/aworkflow.h>

class CAlgorithmSettingsModel : public QAbstractItemModel
{
public:
  CAlgorithmSettingsModel(AWorkflow& workflow);
  void saveSettings(int row, QString filename);
  void loadSettings(int row, QString filename);
  QModelIndex index(int row, int column,QModelIndex& parent);
  QModelIndex parent(QModelIndex& index); 
  int rowCount(QModelIndex& parent);
  int columnCount(QModelIndex& parent);
  QVariant data(QModelIndex& index, int role);
  Qt::ItemFlags flags(QModelIndex& index);
  bool setData(QModelIndex& index, QVariant& value, int role);
};
#endif //CALGORITMSETTINGSMODEL_H
