#ifndef CDATASETSELECTOR_H
#define CDATASETSELECTOR_H

#include <QListWidget>

#include <io/CInputDataSet.h>
#include <workflow/workflow/ccontextdatastore.h>

namespace Ui {
  class CDatasetSelector;
}

class CDatasetSelector : public QWidget
{
  Q_OBJECT

public:
  explicit CDatasetSelector(QWidget* parent = nullptr);
  ~CDatasetSelector();

  void newContextStore(CContextDataStore* dataStore);
  void clear();

signals:
  void currentDataStoreChanged(CContextDataStore* dataStore);
  void dataStoreDeleted(CContextDataStore* dataStore);
  void newDataStoreRequested();

private:
  Ui::CDatasetSelector* ui;

private slots:
  void onCurrentItemChanged(QListWidgetItem *current, QListWidgetItem *previous);
  void addDataset();
  void removeDataset();
};

#endif // CDATASETSELECTOR_H
