#ifndef CDATASETSELECTORITEM_H
#define CDATASETSELECTORITEM_H

#include <workflow/workflow/ccontextdatastore.h>

class CDatasetSelectorItem : public QListWidgetItem
{
public:
  CDatasetSelectorItem(CContextDataStore* dataStore, QString text);
  CContextDataStore* getDataStore();

private:
  CContextDataStore* mpDataStore = nullptr;
};

#endif // CDATASETSELECTORITEM_H
